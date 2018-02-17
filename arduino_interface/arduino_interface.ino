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
#include <Servo.h>

Sabertooth Steering= Sabertooth(128, Serial1);
Sabertooth Actuators = Sabertooth(129, Serial1);


ros::NodeHandle  nh;



// Setup Motor Controllers

bool DEBUG = true;

// Setup Brake
int brakeEncoderPin = 0;
int brakeServoPin = 9;
float brakeAlpha = 5.;
float brakeBasePos = 0;
bool brakeInvert = false;
float brakeInitialPos = 0;
float brakeMotorNum = 1;

MotorController brakeController = MotorController(brakeEncoderPin, brakeServoPin, 
                                                  brakeAlpha, brakeBasePos,  brakeMotorNum, 
                                                  DEBUG, Actuators, "brake"); // brake


// Setup Gear Controller
int gearEncoderPin = 1;
int gearServoPin = 10;
float gearAlpha = 5;
float gearBasePos = 0;
bool gearInvert = false;
float gearInitialPos = 470;
int gearMotorNum = 2;

// Gear postions
int parkGearPos = 470 - gearInitialPos;
int neutralGearPos = 310 - gearInitialPos;
int  reverseGearPos = 380 - gearInitialPos;
int driveGearPos = 260 - gearInitialPos;

MotorController gearController = MotorController(gearEncoderPin, gearServoPin, 
                                                 gearAlpha, gearBasePos,  gearMotorNum, 
                                                  DEBUG, Actuators, 
                                                  "gear"); // gear selector

// Setup Steering Controller

int steeringEncoderPin = 2; 
int steeringServoPin = 11;  
float steeringAlpha = 5;
float steeringBasePos = 127;
bool steeringInvert = false;
float steeringInitialPos = 127;
int steeringMotorNum = 2;

MotorController steeringController = MotorController(steeringEncoderPin , steeringServoPin, 
                                                      steeringAlpha, steeringBasePos, 
                                                      steeringMotorNum, DEBUG, 
                                                      Steering, "steering");




// Setup Throttle Controller

int throttleServoPin = 22;

float throttleInitialPos = 127;
// Setup Ignition
int ignitionPin = 4;


// Setup Starter
int starterPin = 3;


// Setup ROS

// DEFINE Callbacks

// Throttle servo setup
Servo throttleServo;

void throttle_cb( const std_msgs::Int16& throttle_cmd){ 
  throttleServo.write(map(throttle_cmd.data, 0, 255, MIN_THROTTLE, MAX_THROTTLE));
}

void steering_cb( const std_msgs::Int16& steering_cmd){
  steeringController.desP = steering_cmd.data;
}

void ignition_cb( const std_msgs::Int16& ignition_cmd){
  if (ignition_cmd.data > 0) {
    digitalWrite(ignitionPin, HIGH);
  } else {
    digitalWrite(ignitionPin, LOW);
  }
}

void gear_cb(const std_msgs::Int16& gear_cmd){

  int newGear = gear_cmd.data;

  float setPoint = -1;
  
  switch (newGear) {
    case 80: // P
      setPoint = parkGearPos;
      break;
    case 78: // N
      setPoint = neutralGearPos;
      break;
    case 82: // R 
      setPoint = reverseGearPos;
      break;
    case 86: // D
      setPoint = driveGearPos;
      break;
  }

  if (setPoint > 0) { 
    gearController.desP = setPoint;
  }
}

void brake_cb(const std_msgs::Int16& brake_cmd){
  brakeController.desP = brake_cmd.data;
}

void starter_cb(const std_msgs::Int16& starter_cmd) {
    if (starter_cmd.data > 0) {
    digitalWrite(starterPin, HIGH);
  } else {
    digitalWrite(starterPin, LOW);
  }
}

//



ros::Subscriber<std_msgs::Int16> sub_throttle("throttle", &throttle_cb);
ros::Subscriber<std_msgs::Int16> sub_steering("steering", &steering_cb);
ros::Subscriber<std_msgs::Int16> sub_ignition("ignition", &ignition_cb);
ros::Subscriber<std_msgs::Int16> sub_gear("gear", &gear_cb);
ros::Subscriber<std_msgs::Int16> sub_brake("brake", &brake_cb);
ros::Subscriber<std_msgs::Int16> sub_starter("starter", &starter_cb);




void setup(){

  // Setup Serial1
  Serial1.begin(9600);

  brakeController.basePos = brakeBasePos;  
  gearController.basePos = gearBasePos;
  steeringController.basePos = steeringBasePos;

  // Setup Throttle.
  throttleServo.attach(throttleServoPin); // throttle on pin 22
  throttleServo.write(map(throttleInitialPos, 0, 255, MIN_THROTTLE, MAX_THROTTLE));
  pinMode(ignitionPin, OUTPUT);
  pinMode(starterPin, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub_throttle);
  nh.subscribe(sub_steering);
  nh.subscribe(sub_ignition);
  nh.subscribe(sub_gear);
  nh.subscribe(sub_brake);  
  nh.subscribe(sub_starter);  



}


void listenToSerial()
{
 while (Serial.available() > 0) {
    char select = Serial.read();
    int pos = Serial.parseInt();
    String tmp = Serial.readString(); // Try to read any junk after..... 
    
    switch (select)
    {
      case 'g':
        gearController.desP = pos;
        break;
      case 'b':
        brakeController.desP = pos;
        break;
      case 's':
        steeringController.desP = pos;
        break;
    }
  }
}


void loop(){
  nh.spinOnce();
  brakeController.update_motor(); 
  gearController.update_motor();
  steeringController.update_motor();
 
  delay(15);
}
