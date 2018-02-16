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
#define address 0x80

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
  Serial.begin(115200); 
}

void update_motor(int desP, char motor) {
  //desP is a number between 0 and 255
  int mPos;
  float output;
  if (motor=='A') {
    mPos = analogRead(mAEnc);  
  } else {
    mPos = analogRead(mBEnc);
  }
  map(mPos, 0, 1023, 0, 255);
  int err =desP - mPos;
  if (err >= 5 or err <= 5){
    if (motor=='A') {
      roboclaw.ForwardM1(address,0);
    } else {
      roboclaw.ForwardM2(address,0);
    }
  } else if (err < 5) {
    if (motor=='A') {
      output = alphaA * err;
      roboclaw.BackwardM1(address, int(output));
    } else {
      output = alphaB * err;
      roboclaw.BackwardM2(address, int(output));
    }    
  } else if (err > 5) {
    if (motor=='A') {
      output = alphaA * err;
      roboclaw.ForwardM1(address, int(output));
    } else {
      output = alphaB * err;
      roboclaw.ForwardM2(address, int(output));
    }   
  }
}

void loop() {
  update_motor(100, 'A');
  update_motor(100, 'B');
  delay(200);
}
