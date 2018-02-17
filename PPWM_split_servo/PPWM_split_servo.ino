
#include <Servo.h>
Servo mAservo;  // create servo object to control a servo
Servo mBservo;  // create servo object to control a servo
int mAEnc = 0;
int mBEnc = 4;
float alphaA = 5.;
float alphaB = 0.5;
 

void setup() {
  Serial.begin(115200); 
  mAservo.attach(9);
  mBservo.attach(10);
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

  //map(mPos, 0, 1023, 0, 255);
  Serial.println(mPos);
  int err =desP - mPos;
  //Serial.println
  if (err <= 5 and err >= -5){
    if (motor=='A') {
      mAservo.write(90);
      Serial.println("no err");
    } else {
      mBservo.write(90);
    }
  } else if (err < -5) {
    if (motor=='A') {
      output = 90.0 + ((alphaA * err) * (85./1024.));
      //Serial.println(output);
      if (output <=5) {
        output = 5;
      }
      mAservo.write(int(output));
    } else {
      output = alphaB * err;
      //roboclaw.BackwardM2(address, int(output));
    }    
  } else if (err > 5) {
    if (motor=='A') {
      output = 90.0 + (alphaA * err) * (85./1024.);
      //Serial.println(output);
      if (output >=175) {
        output = 175;
      }
      mAservo.write(int(output));
    } else {
      output = alphaB * err;
      //roboclaw.ForwardM2(address, int(output));
    }   
  }
}

void loop() {
  update_motor(600, 'A');
  //update_motor(100, 'B');
  delay(100);
}
