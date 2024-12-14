
#include <Servo.h>
#include <SoftwareSerial.h>
// #include <VarSpeedServo.h>

// Define pins for SoftwareSerial
SoftwareSerial piSerial(10,11); // RX (5), TX (6)

// Servo motor setup
Servo motor;
// VarSpeedServo motor;
int pwmPin = 9; // Servo motor signal pin
int currentAngle = 180; // Default starting position
int sensorPin = A0;
bool gripped = false;


float getForce(){
  int analog = analogRead(sensorPin);
  float force = 16.854*exp(0.0047*analog)-16.85;
  return force;
}

bool inRange(){
  if(currentAngle>=0 && currentAngle<=180){
    return true;
  }
  else{
    return false;
  }
}

float getThickness(int angle){
  
}

void close() {
  float threshold = 130;
  int delta = 2;

  while (!gripped) {
    float current_force = getForce();
    Serial.print("Force: ");
    Serial.println(current_force);

    if (current_force < threshold) {
      // if (currentAngle - delta >= 0) { // Ensure angle is in range
        motor.write(currentAngle - delta);
        currentAngle -= delta;
        Serial.print("Current angle: ");
        Serial.println(currentAngle);
      // } else {
      //   Serial.println("Reached minimum angle limit!");
      //   break;
      // }
    } else {
      gripped = true;
      Serial.println("Closed");
      return;
    }
  }
}

void open() {
  if (!gripped) {
    Serial.println("Error: gripped is false. Cannot open.");
    return;
  }
  
  float threshold = 7;
  int delta = 1;

  while (gripped) {
    float current_force = getForce();
    Serial.print("Force: ");
    Serial.println(current_force);

    if (current_force > threshold) {
      // if (currentAngle + delta <= 180) {
        Serial.print("Moving motor to angle: ");
        Serial.println(currentAngle + delta);
        motor.write(currentAngle + delta);
        currentAngle += delta;
        Serial.print("Current angle: ");
        Serial.println(currentAngle);
      // } else {
      //   Serial.println("Reached maximum angle limit!");
      //   break;
      // }
    } else {
      gripped = false;
      Serial.println("opened");
      return;
    }
  }
}

void shutdown(){
  while(true){

  }
}

void setup() {
  motor.attach(pwmPin); // Attach servo to PWM pin
  Serial.begin(9600);   // Serial communication for debugging
  piSerial.begin(9600); // Communication with Raspberry Pi

  motor.write(180); // Move servo to default position
  Serial.println("System initialized and homed to 180 degrees.");
  delay(1000);
}

void loop() {
  // Check for data from Raspberry Pi
  if (piSerial.available()) {
    // delay(50); // Small delay to ensure data is received
    String input = piSerial.readStringUntil('\n'); // Read input until newline
    input.trim(); // Remove leading/trailing whitespace

    Serial.print("Received from Pi: ");
    Serial.println(input); // Debug print received input

    // Convert String to char* for strcmp comparison
    char inputChar[input.length() + 1];
    input.toCharArray(inputChar, sizeof(inputChar));
    Serial.println(inputChar);
    // Use strcmp to compare the received input
    if (strcmp(inputChar, "1") == 0) { // If input is "1"
      Serial.println("Closing...");
      // close(); // Call the 'close' function (assumes it's defined elsewhere)
      piSerial.println("OK"); // Send response back to Pi
    } 
    else if (strcmp(inputChar, "0") == 0) { // If input is "0"
      Serial.println("Opening...");
      // open(); // Call the 'open' function (assumes it's defined elsewhere)
      piSerial.println("OK"); // Send response back to Pi
    } 
    else {
      Serial.println("Unknown command received.");
      piSerial.println("error"); // Inform Pi of invalid command
    }
  }
  else{
    // motor.write(0);
    // delay(3000);
    // motor.write(180);
    // delay(3000);
    
    // Serial.println("waiting pi");
    // piSerial.println("fk");
    // delay(1000);
    // close();
    // delay(1000);
    // open();
    // int analog = analogRead(sensorPin);
    // Serial.println(analog);
    // Serial.println(getForce());

   }
}


