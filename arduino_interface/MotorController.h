#include <Sabertooth.h>

int hyst = 20;

class MotorController
{
  public: 
     MotorController(int MEP, int MSP, float A, float bP, int motorn, bool d, 
     Sabertooth &sab, String nams)
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


  mPos = map(mPos, 0, 1023, 0, 255);
  if (debug) {  Serial.println(mPos); }
  
  int err =desP - mPos;
  //Serial.println
  if (err <= hyst and err >= -hyst){
    if (debug) {Serial.println("no err");}
    output = 90;    
  } else if (err < -hyst) {
      output = 80.0 + ((alpha * err) * (85./255.));
       
      if (output <= 5) { output = 5; }
      
      if (debug) {Serial.println(output);}
     
  } else if (err > hyst) {
      
      output = 100.0 + (alpha * err) * (85./255.);
      
      if (output >=175) { output = 175; }

      
  }
  output = map(int(output), 0, 180, -127, 127);
  int int_output = int(output);
  
  if (debug) {Serial.println(output);}

  saber->motor( motorNumber, int_output);
  
};

