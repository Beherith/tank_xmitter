// This is the tank Transmitter code, Select Arduino Nano with ATMEGA328p processor when compiling
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h" //for the NRF library

uint8_t address[] = "Tiger";
uint8_t packet[16];
/* //todo: use a shared struct instead of pins :/
struct DataPacket{
	uint8_t leftfw;
	uint8_t leftback;
	uint8_t rightfw;
	uint8_t rightback;
	uint8_t turretleft;
	uint8_t turretright;
	uint8_t turretup;
	uint8_t turretdown;
	uint8_t light;
	uint8_t smoke;
	uint8_t fire;
	uint8_t unused1;
	uint8_t unused2;
	uint8_t unused3;
	uint8_t unused4;
	uint8_t sumfields;
};
struct DataPacket packet = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
*/

const int yaxis     = A7;
const int xaxis     = A6;
const int turretleftbtn = 8;
const int turretrightbtn= 9;
const int turretupbtn = 4;
const int turretdownbtn = 5;
const int firebtn   = 6;
const int smokebtn    = 7;
//const int lightbtn    = 7;
//const int fbled = 8;
const int redled = A0;
const int commonled = A1;
const int greenled = A2;
const int debugin = A3;
const int debugout = A4;
const int softpotpin = A5;
bool debug = 1;


//These are unused in the transmitter, just tells where to pack the bytes
const int leftfwpin   = 12;
const int leftbackpin = 11;
const int rightfwpin  = 10;
const int rightbackpin  = 9;
const int turretleftpin = 8;
const int turretrightpin= 7;
const int turretuppin = 6;
const int turretdownpin = 5;
const int firepin   = 4;
const int shotfeedbackpin = 3; 
const int lightpin    = 2;
const int smokepin    = 44;
RF24 radio(3, 2); //CE & CS true!

//-----------------------------------------Important constants--------------------------------

unsigned long lastshottime = millis();
const unsigned long shotdelay = 2000;

unsigned long lastpacketsent = millis();
const unsigned long packettimeout = 100;

float deadzone=0.05; //dead zone of the sticks, so shit doesnt judder at rest
float centerx =0.5; //default center point of the sticks [0;1] calibrated at startup
float centery=0.5;
float exponential = 0.5; //[0 ; 1] controls throttle exponential
const int inertia  = 0.1; //implements inertia in the controls, as rapid forward backwards swings could damage the tank
float softness = 0.5; //i forgot what this does, but it seems pretty important

float prevx=0; //store previous axis value for smoothing
float prevy=0;

int mindrive = 10; //minimum PWM value to output, to prevent very low currents where the motors dont turn, just draw current

const int turretrotspeed = 0; // lower values are faster speeds! [255;0]
//Turret rotation is currently not analog, although the reciever supports it

const int turretelspeed = 0; // lower values are faster speeds! [255;0]
// turret elevation is not analog, but the 
const bool decaymode = 0; //slow, shorts =0, fast, coasts = 1
#define SLOW 0
#define FAST 1 
//--------------------------------------------------------------------------------------------
float calibrateaxis(int pin){
  prec_analog(pin,32);
  return prec_analog(pin,32);
  }
void setup(){
  pinMode(debugout,OUTPUT);
  pinMode(debugin,INPUT_PULLUP);
  digitalWrite(debugout,LOW);
  if (digitalRead(debugin) == 1){
    debug = 1;
  }
  Serial.begin(115200);
  printf_begin();
  Serial.println("Tank transmitter, attempting to set up radio...");
  if( radio.begin()) Serial.println("radio.begin() Success");
  else Serial.println("radio.begin() FAILED");
  radio.setChannel(125);
  radio.setPayloadSize(sizeof(packet));
  radio.setPALevel(RF24_PA_MAX);
  if( ! radio.setDataRate(RF24_250KBPS)){
    if (debug) Serial.println("Failed to set data rate to 250kbps");
  }
  radio.openWritingPipe(address);
  radio.printDetails();
  Serial.println("If you do not see radio details above, something is very wrong!");
  centerx=calibrateaxis(xaxis);
  centery=calibrateaxis(yaxis);
  prevx=0.0;
  prevy=0.0;
  pinMode(softpotpin,INPUT_PULLUP);
  for (byte i =0;i<16;i++) packet[i]=0;
  pinMode(turretleftbtn, INPUT_PULLUP);
  pinMode(turretrightbtn, INPUT_PULLUP);
  pinMode(turretupbtn, INPUT_PULLUP);
  pinMode(turretdownbtn, INPUT_PULLUP);
  pinMode(firebtn, INPUT_PULLUP);
  pinMode(smokebtn, INPUT_PULLUP);
  //pinMode(lightbtn, INPUT_PULLUP);
  
  pinMode(redled,OUTPUT);
  pinMode(commonled,OUTPUT);
  digitalWrite(commonled, LOW);
  digitalWrite(redled,HIGH);
  pinMode(greenled,OUTPUT);
  digitalWrite(greenled,LOW);
}

float prec_analog(int pin, const byte samples){
  int tmp = 0;
  for (byte b=0; b<samples;b++){
    tmp = tmp+ analogRead(pin);
  }
  return ((float)tmp)/(1023.0*samples); //[0-1]
}

  
float gate(float x, float center, float scale){
  if (abs(x-center) <= deadzone){
    x= center;
  }else{
    if (x<center) x = x+deadzone;
    else x = x -deadzone;
  }
  
  x = 2*(x-center) * scale;
  x = min(1.0,max(-1.0,x));
  x = exponential * x + (1-exponential)*x*x*x;

  return x;
  }

void ramp(int pin, int target, int inertia){
  //Add some inertia to the system, like maybe a full sec at full swing?
  //To prevent shoot-through, targets of 0 will be set immedatiely!
  if (target == 0){
    if (decaymode == SLOW){
      packet[pin] = 255;
    }
    if (decaymode == FAST){
      packet[pin] = 0;
      }
    return;
  }
 
  if (packet[pin] - target > 2*inertia){ // slow down ramp is faster
    packet[pin] = packet[pin] - 2*inertia;
  }else if(target - packet[pin] > inertia){
    packet[pin] = packet[pin]+inertia;    
  }else{
    packet[pin]=target;
  }
}

void drivetrack(int fwpin, int revpin, int fwamnt, int revamnt){
  #define RAMP 20
  const int mul = 255;
  //COAST if all 0:
  if (fwamnt == 0 && revamnt == 0){
    packet[fwpin ] = 0;
    packet[revpin] = 0;
    return;
  }else if( fwamnt > 0 && revamnt > 0){
    packet[lightpin]=32;
    if (debug) Serial.println("A track is being driven in both directions!");
    packet[fwpin ] = 0;
    packet[revpin] = 0;
    return;
  }else{
    int gopin = fwpin;
    int stoppin = revpin;
    int targetspeed = fwamnt;
    int prevspeed = packet[gopin];
    if (revamnt >0){
      gopin = revpin;
      stoppin = fwpin;
      targetspeed = revamnt;
      prevspeed = packet[gopin];
      }
    if (targetspeed < mindrive) targetspeed = mindrive;
    packet[stoppin]= 255;
    packet[gopin] = 255 - targetspeed;
  } 
  
}


void sticks(){

  float rawx = prec_analog(xaxis,16);
  float rawy = prec_analog(yaxis,16);
  float softness = prec_analog(softpotpin,4);
  softness = softness *softness;
  
  float x = gate(rawx, centerx, 1.3);
  float y = gate(rawy, centery,1.5);
  //some softening of the sticks
  //use linear softness instead of exponential:
  
  softness=0.17;
  x = prevx + constrain(x-prevx,-1 * softness, softness);
  y = prevy + constrain(y-prevy,-1 * softness, softness);
  
  prevx=x;
  prevy=y;
  
  x = x * (1-abs(y*0.5));
  if (debug){
    Serial.print("X-axis:");
    Serial.println(x);
    Serial.print("y-axis:");
    Serial.println(y);
  }
  const int mul = 255;
  int left_reverse = mul * max(0, min(1.0,   (x - y)));
  int left_forward = mul * max(0, min(1.0, - (x - y)));
  int right_reverse= mul * max(0, min(1.0, - (x + y)));
  int right_forward= mul * max(0, min(1.0,   (x + y)));

  drivetrack(leftfwpin,leftbackpin,left_forward,left_reverse);
  drivetrack(rightfwpin,rightbackpin,right_forward,right_reverse);
  }

void printpacket(){
  if (!debug) return;
      for (byte i =0; i<sizeof(packet); i++){
        Serial.print(i);
        Serial.print(":");
        Serial.print(packet[i]);
        Serial.print("\t" );
      }
      Serial.println();
  }

void loop(void){
  packet[lightpin] = 255; //keep the lights on!
  sticks();
  
  if (digitalRead(turretleftbtn) == 0){
    packet[turretleftpin] = turretrotspeed;
    packet[turretrightpin] = 255; //use slow decay for breaking
  }else{
    packet[turretleftpin] =255;//use slow decay for breaking
    if (digitalRead(turretrightbtn) == 0) packet[turretrightpin] = turretrotspeed;
    else  packet[turretrightpin] = 255;//use slow decay for breaking
  }


  if (digitalRead(turretupbtn) == 0){
    packet[turretuppin] = turretelspeed;
    packet[turretdownpin] = 255;
  }else{
    packet[turretuppin] =255;
    if (digitalRead(turretdownbtn) == 0) packet[turretdownpin] = turretelspeed;
    else  packet[turretdownpin] = 255;
  }


  if (digitalRead(firebtn) == 0){
    packet[firepin] = 255;
  }else{
    packet[firepin] =0;
  }

  
  if (digitalRead(smokebtn) == 0){
    packet[1] = 255;
  }else{
    packet[1] =0;
  }

  printpacket();
  bool ok = radio.write(packet,sizeof(packet));
  if (ok){   
    digitalWrite(greenled,HIGH); 
    if (debug) Serial.print("ok\n\r");
  }
  else{
    digitalWrite(greenled,LOW);  
    if (debug) Serial.print("failed\n\r");
  }
  Serial.print(millis()-lastpacketsent);    
  Serial.println("ms");
  
  lastpacketsent = millis();
  delay(25);
}

