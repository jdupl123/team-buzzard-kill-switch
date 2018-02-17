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

#include "MotorController.h"
#include <ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Char.h>

#include <SoftwareSerial.h>

ros::NodeHandle  nh;



// Setup Motor Controllers

bool DEBUG = true;

// Setup Brake
int brakeEncoderPin = 0;
int brakeServoPin = 9;
float brakeAlpha = 5.;
float brakeBasePos = 380;

MotorController brakeController = MotorController(brakeEncoderPin, brakeServoPin, 
                                                  brakeAlpha, brakeBasePos, 
                                                  DEBUG); 

// Setup Gear Controller
int gearEncoderPin = 1;
int gearServoPin = 10;
float gearAlpha = 5;
float gearBasePos = 0;
                                       
MotorController gearController = MotorController(gearEncoderPin, gearServoPin,
                                                 gearAlpha, gearBasePos, 
                                                 DEBUG); // gear selector
// Setup Steering Controller

int steeringEncoderPin = 6; // NOT SURE ACTUALLY
int steeringServoPin = 12;  // NOT SURE ACTUALLY
float steeringAlpha = 5;
float steeringBasePos = 5;

//MotorController steeringController = MotorController(steeringEncoderPin, steeringServoPin,
//                                                     steeringAlpha, steeringBasePos);









// Setup ROS

// DEFINE Callbacks

// Throttle servo setup
Servo servo;

void throttle_cb( const std_msgs::Int16& throttle_cmd){ 
  servo.write(map(throttle_cmd.data, 0, 255, MIN_THROTTLE, MAX_THROTTLE));
}

void steering_cb( const std_msgs::Int16& steering_cmd){
  // Do whatever
  //print(steering_cmd.data) 
}

void ignition_cb( const std_msgs::Int16& ignition_cmd){
  if (ignition_cmd.data > 0) {
    digitalWrite(3, HIGH);
  } else {
    digitalWrite(3, LOW);
  }
}

void gear_cb(const std_msgs::Int16& gear_cmd){
  
}

void brake_cb(const std_msgs::Int16& brake_cmd){
  
}


//



ros::Subscriber<std_msgs::Int16> sub_throttle("throttle", &throttle_cb);
ros::Subscriber<std_msgs::Int16> sub_steering("steering", &steering_cb);
ros::Subscriber<std_msgs::Int16> sub_ignition("ignition", &ignition_cb);
ros::Subscriber<std_msgs::Int16> sub_throttle("gear", &gear_cb);
ros::Subscriber<std_msgs::Int16> sub_throttle("brake", &brake_cb);




void setup(){

  nh.initNode();
  nh.subscribe(sub_throttle);
  nh.subscribe(sub_steering);
  nh.subscribe(sub_ignition);
  
  servo.attach(22); // throttle on pin 22
  pinMode(3, OUTPUT);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
