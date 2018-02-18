
#include <Sabertooth.h>

Sabertooth Steering= Sabertooth(128, Serial1);
Sabertooth Actuators = Sabertooth(129, Serial1);

bool DEBUG = true;
int hyst = 10;

class MotorController
{
  public: 
     MotorController(int MEP, int MSP, float A, float bP, int motorn, bool d, Sabertooth &sab, String nams)
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
      nams = nam;
     };
     void update_motor();

     String nam;
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
      output = 70.0 + ((alpha * err) * (85./1024.));
       
      if (output <= 5) { output = 5; }
      
      if (debug) {Serial.println(output);}
     
  } else if (err > hyst) {
      
      output = 110.0 + (alpha * err) * (85./1024.);
      
      if (output >=175) { output = 175; }

      
  }
  output = map(int(output), 0, 180, 127, -127);
  
  if (debug) {Serial.println(output);}

  saber->motor( motorNumber, output);
  
};


MotorController brakeController = MotorController(0, 9, 5., 0,  1, DEBUG, Actuators, "brake"); // brake
MotorController gearController = MotorController(1, 10, 5., 0,  2, DEBUG, Actuators, "gear"); // gear selector
MotorController steeringController = MotorController(2 , 11, 10, 0, 2, DEBUG, Steering, "steering"); // to check

void setup() {
  Serial1.begin(9600);
  //Actuators.autobaud();
  
  //Steering.autobaud();
  Serial.begin(9600);
  
  //Serial.begin(115200); 

  // steeringController.attach_servo();
  //gearController.basePos = 0;
  

};


void loop() { 
  //brakeController.desP = 200;
  //gearController.desP = 250;
  steeringController.desP = 850;
  //brakeController.update_motor(); 
  steeringController.update_motor();
  
  //gearController.update_motor(); 
  //brakeController.update_motor();
  //Serial.println("Hello");
  delay(150);
}


