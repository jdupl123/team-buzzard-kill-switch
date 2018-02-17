
#include <Sabertooth.h>

Sabertooth Steering = Sabertooth(128);
Sabertooth Actuators = Sabertooth(129);

bool DEBUG = false;
int hyst = 10;

class MotorController
{
  public: 
     MotorController(int MEP, int MSP, float A, float bP, int motorn, bool d, Sabertooth &sab)
     {
      motorEncoderPin = MEP;
      motorServoPin = MSP;
      alpha = A;
      debug = d;
      maxPos = 1024;
      basePos = bP;
      desP = basePos;
      motorNumber = motorn;
      saber = &sab;
     };
     void update_motor();

     float maxPos; // max setting
     float basePos; // ADC measurement when fully retracted.
     float desP; // Set desired position. 
     int motorNumber;
     Sabertooth* saber;
     
  private:
     int ST;

     int motorEncoderPin;
     float alpha;
     int motorServoPin;
     bool debug;

};


void MotorController::update_motor()
{
 //desP is a number between 0 and 255
  int mPos;
  float output;
  mPos = analogRead(motorEncoderPin);  

  // Add offset
  desP = desP + basePos;

  // clip desP to min max allowed position.
  if ( desP > maxPos) {  desP = maxPos; };


  //map(mPos, 0, 1023, 0, 255);
  if (debug) {  Serial.println(mPos); }
  
  int err =desP - mPos;
  //Serial.println
  if (err <= hyst and err >= -hyst){
    if (debug) {Serial.println("no err");}
    output = 90;    
  } else if (err < -hyst) {
      output = 80.0 + ((alpha * err) * (85./1024.));
       
      if (output <= 5) { output = 5; }
      
      if (debug) {Serial.println(output);}
     
  } else if (err > hyst) {
      
      output = 100.0 + (alpha * err) * (85./1024.);
      
      if (output >=175) { output = 175; }

      if (debug) {Serial.println(output);}
      
  }

  
  saber->motor( motorNumber,map(int(output), 0, 180, -127, 127));
  
};


MotorController brakeController = MotorController(0, 9, 5., 0,  2,DEBUG, Actuators); // brake
MotorController gearController = MotorController(1, 10, 5., 0,  1, DEBUG, Actuators); // gear selector
MotorController steeringController = MotorController(2 , 11, 5, 0, 1, DEBUG, Steering); // to check

void setup() {
  SabertoothTXPinSerial.begin(9600);
  
  Actuators.autobaud();
  
  Steering.autobaud();
  
  //Serial.begin(115200); 

  // steeringController.attach_servo();
  gearController.basePos = 380;
  

};


void loop() { 
  //brakeController.desP = 800;
  gearController.desP = 200;
  //brakeController.update_motor(); 
  gearController.update_motor();
  
  delay(100);
}


