
#include <Servo.h>

class MotorController
{
  public: 
     MotorController(int MEP, int MSP, float A, float bP, float iP, bool inv, bool d)
     {
      /*
       *  
       */
      motorEncoderPin = MEP;
      motorServoPin = MSP;
      alpha = A;
      debug = d;
      maxPos = 1024;
      basePos = bP;
      invert = inv;
      initialPosition = iP; 
      desP = initialPosition;
      hyst = 10;
     }
     void update_motor();
     
     void attach_servo()
     {
      servo.attach(motorServoPin);
      
     };

     float maxPos; // max setting
     float basePos; // ADC measurement when fully retracted.
     float desP; // Set desired position.
     float initialPosition;
     bool invert; 
     int hyst;
     
  private:
     Servo servo;
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

 // Apply invert if neccesary.
  if (invert) { desP = map(desP, 0, 255, 255, 0);}

  // Apply scaling
  desP = map(desP, 0, 255, 0, 1023);

 
  // Add offset
  desP = desP + basePos;

  // clip desP to min max allowed position.
  if ( desP > maxPos) {  desP = maxPos; };


  if (debug) {  Serial.println(mPos); }
  
  int err =desP - mPos;
  //Serial.println
  if (err <= hyst and err >= -hyst){
    if (debug) {Serial.println("no err");}    
    output = 90;    
  } else if (err < -hyst) {
      output = 80.0 + ((alpha * err) * (85./1024.));
      if (debug) {Serial.println(output);}
      
      if (output <=5) { output = 5; }
     
  } else if (err > hyst) {
      
      output = 100.0 + (alpha * err) * (85./1024.);
      
      if (debug) {Serial.println(output);}
      
      if (output >=175) { output = 175; }
      
  }

  servo.write(int(output));
  
};
