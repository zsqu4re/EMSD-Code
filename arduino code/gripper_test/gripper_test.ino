#include <Servo.h>
#include <math.h> // For exp()

Servo myservo;  // create servo object to control a servo
const float circumference = 62.8; // mm
const int fsrPin = 0;  // FSR and 10K pulldown connected to A0
int fsrReading;

// Function to rotate the servo to a certain degree
void rotate(int degree) {
  if (degree < 0) degree = 0;
  if (degree > 180) degree = 180; // Ensuring within servo bounds
  myservo.write(degree);
  delay(1000 * (degree / 60.0) * 0.25); // Proper floating-point calculation
}

// Function to convert linear distance to servo rotation
void linear(float dis) {
  float degree = (dis / (2 * circumference)) * 360.0;
  rotate((int)degree); // Casting to int for servo control
}

// Function to calculate force based on FSR reading
float calculateForce(int fsrReading) {
  float mass = 16.854 * exp(0.0047 * fsrReading);
  return mass * 9.8; // Force in Newtons
}

void setup() {
  myservo.attach(9); // Attach servo to pin 9
  myservo.write(0);  // Set servo to 0 degrees
  delay(2000);       // Wait for servo to stabilize
}

void loop() {
  fsrReading = analogRead(fsrPin); 
  float force = calculateForce(fsrReading);

  // Move servo while force is below threshold

  delay(1000);
  while (force < 50.0) {
    Serial.println("start gripping");
    linear(-20.0);
    force = calculateForce(analogRead(fsrPin)); // Update force reading
  }

  delay(1000);
  linear(0); // Reset servo position
}
