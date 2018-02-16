//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);   
RoboClaw roboclaw(&serial,10000);
int mAEnc = 3;
int mBEnc = 4;
float alphaA = 0.5;
float alphaB = 0.5;
float output=0;
#define address 0x80

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
  Serial.begin(115200); 
}

void update_motors(int desPA, int desPB) {
  //desPA nd desPB are numbers between 0 and 255
  int mAPos = analogRead(mAEnc);
  mAPos = map(mAPos, 0, 1023, 0, 180);
  int mBPos = analogRead(mBEnc);
  mBPos = map(mBPos, 0, 1023, 0, 180);
  int eA = desPA - mAPos; //error in motor A
  int eB = desPB - mBPos; //error in motor B
  if (eA >= 5 or eA <= 5){
    roboclaw.ForwardM1(address,0);
  } else if (eA < 5) {
    output = alphaA * eA;
    roboclaw.BackwardM1(address, int(output));
  } else if (eA > 5) {
    output = alphaA * eA;
    roboclaw.ForwardM1(address, int(output));
  }
  if (eB >= 5 or eB <= 5){
    roboclaw.ForwardM2(address,0);
  } else if (eB < 5) {
    output = alphaB * eB;
    roboclaw.BackwardM2(address, int(output));
  } else if (eA > 5) {
    output = alphaB * eA;
    roboclaw.ForwardM2(address, int(output));
  }
  
  
}

void loop() {
  update_motors(100, 100);
  delay(2000);
}
