
int STARTER_PIN_OUT= 32;

void setup() {
  // put your setup code here, to run once:
  // initialize serial:
  Serial.begin(9600);
  
}

void loop() {
  while (Serial.available() > 0) {
    
    // Grab an input, we expect a 1
    int input = Serial.parseInt();
    String blah = Serial.readString();

    if (input == 1) 
    {
      // Set starter pin to high  

      digitalWrite(STARTER_PIN_OUT, HIGH);
      // Wait for car to start second
      delay(1);

      // Set starter pin to low
      digitalWrite(STARTER_PIN_OUT, LOW);
    }
    
  }
}
