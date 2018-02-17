/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#define MIN_THROTTLE 140
#define MAX_THROTTLE 40

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

#include <SoftwareSerial.h>
#include "RoboClaw.h"

ros::NodeHandle  nh;

// Throttle servo setup
Servo servo;
RoboClaw roboclaw(&serial,10000);

int mAEnc = 3;
int mBEnc = 4;
float alphaA = 0.5;
float alphaB = 0.5;
#define address 0x80

void throttle_cb( const std_msgs::Int16& throttle_cmd){ 
  servo.write(map(throttle_cmd.data, 0, 255, MIN_THROTTLE, MAX_THROTTLE));
}

void transmission_cb( const std_msgs::Char& transmission_cmd){
  update_motor(brake_cmd.data, 'A');
}

void brake_cb( const std_msgs::Int16& brake_cmd){
  update_motor(brake_cmd.data, 'B');
}

void steering_cb( const std_msgs::Int16& steering_cmd){
  // Do whatever
  print(steering_cmd.data) 
}

void ignition_cb( const std_msgs::Int16& ignition_cmd){
  if (ignition_cmd.data > 1) {
    digitalwrite(3, HIGH);
  } else {
    digitalwrite(3, LOW);
  }
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

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
  Serial.begin(115200); 
}

ros::Subscriber<std_msgs::Int16> sub("throttle", &throttle_cb);
ros::Subscriber<std_msgs::Char> sub("transmission", &transmission_cb);
ros::Subscriber<std_msgs::Int16> sub("brake", &brake_cb);
ros::Subscriber<std_msgs::Int16> sub("steering", &steering_cb);
ros::Subscriber<std_msgs::Int16> sub("starter", &steering_cb);

void setup(){

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(22); // throttle on pin 22
  pinmode(3, output);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
