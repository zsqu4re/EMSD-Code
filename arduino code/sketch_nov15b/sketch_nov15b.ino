#include <Servo.h>
#include <math.h> // For exp()

Servo myservo;  // Create servo object to control a servo
const float circumference = 62.8; // mm
const int fsrPin = 0;  // FSR and 10K pulldown connected to A0
int fsrReading;

// Function to drive the servo using microseconds
void driveServoMicroseconds(int pulseWidth) {
  if (pulseWidth < 500) pulseWidth = 500;
  if (pulseWidth > 2500) pulseWidth = 2500; // Ensuring within servo bounds
  myservo.writeMicroseconds(pulseWidth);
  delay(1000 * (pulseWidth / 1500.0) * 0.25); // Proper floating-point calculation
}

// Function to convert linear distance to microseconds
void linear(float dis) {
  // Calculate the equivalent pulse width based on distance
  float degree = (dis / (2 * circumference)) * 360.0;
  int pulseWidth = 900 + (int)(degree / 0.15); // Convert degrees to pulse width
  driveServoMicroseconds(pulseWidth); // Drive servo with calculated pulse width
}

// Function to calculate force based on FSR reading
float calculateForce(int fsrReading) {
  float mass = 16.854 * exp(0.0047 * fsrReading);
  return mass * 9.8; // Force in Newtons
}

void setup() {
  myservo.attach(9); // Attach servo to pin 9
  myservo.writeMicroseconds(900);  // Set servo to minimum pulse width
  delay(2000);       // Wait for servo to stabilize
}

void loop() {
  Serial.begin(9600);
  fsrReading = analogRead(fsrPin); 
  float force = calculateForce(fsrReading);
  Serial.println("begin gripping");
  // Move servo while force is below threshold
  while (force < 50.0) {
    linear(-20.0);
    force = calculateForce(analogRead(fsrPin)); // Update force reading
    Serial.println(force);
  }

  delay(1000);
  linear(0); // Reset servo position
}