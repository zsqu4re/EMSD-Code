// #include <Servo.h> 
#include <SoftwareSerial.h>
#include <VarSpeedServo.h>

SoftwareSerial mySerial(5, 6); // RX, TX is 5 , 6

// Servo motor; // Create a Servo object for the motor
VarSpeedServo motor;
int pwmPin = 10; // PWM pin connected to the motor signal
int sensorPin = A0; // Analog pin connected to the force reaction sensor

// Define PWM pulse widths corresponding to 0° and 180° based on the motor's specification
int forceThreshold = 300; // Set the threshold (based on sensor value range)

int currentAngle = 180; // Current angle of motor

// Function to read force sensor value and return as voltage
// float readAnalog() {
//   int rawValue = analogRead(sensorPin); // Read raw sensor value
//   // Serial.println(rawValue);
//   return rawValue;
// }

// // spin the motor for close purpose
// void motorspin_close(int angle){
//   int unit_delta_angle = (angle - currentAngle) / abs(angle - currentAngle);
//   while(currentAngle != angle){
//     motor.write(currentAngle + unit_delta_angle);
//     currentAngle += unit_delta_angle;
//     if (!check_FSR()) {
//       return;
//     }
//   }
// }

// // spin the motor for open purpose
// void motorspin_open(int angle){
//   motor.write(angle);
//   currentAngle = angle;
// }

// bool check_FSR(){
//   // Check FSR sensor
//   int FSR = readAnalog();

//   if (FSR > forceThreshold) { // Compare with threshold
//     Serial.println("FSR exceeds threshold! Motor stopped.");
//     return 0;
//   }
//   else{
//     return 1;
//   }
// }
// // void receive_arduino_command() {
// //   // Check if there is serial input available
// //   if (Serial.available() > 0) {
// //     String input = Serial.readStringUntil('\n'); // Read input until newline
// //     input.trim(); // Remove any leading or trailing whitespace

// //     if (input.length() > 0) {
// //       int angle = input.toInt(); // Convert the input string to an integer

// //       // Validate the angle range (e.g., 0 to 180 degrees)
// //       if (angle < 0){
// //         angle = 0;
// //       } else if(angle > 180){
// //         angle = 180;
// //       }

// //       if(angle < currentAngle){
// //         motorspin_close(angle);
// //       } else{
// //         motorspin_open(angle);
// //       }
// //     }
// //   }
// // }

// int receive_pi_command() {
//   // Check if there is serial input available
//   if (mySerial.available()) {
//     String input = mySerial.readStringUntil('\n'); // Read input from Raspberry Pi
//     input.trim(); // Remove leading/trailing whitespace

//     Serial.print("Received from Pi: ");
//     Serial.println(input); // Debug print received input

//     // Convert to integer and validate angle
//     int angle = input.toInt();
//     if (angle >= 0 && angle <= 180) {
//       motor.write(angle); // Move servo to specified angle
//       currentAngle = angle; // Update current angle
//       Serial.print("Servo moved to angle: ");
//       Serial.println(angle);
//     } else {
//       Serial.println("Invalid angle received. Ignoring.");
//     }
//     return angle; // Move this to the end
//   }
//   return -1; // Return an invalid angle if no data available
// }


void setup() {
  motor.attach(pwmPin); // Attach motor with specified pulse range
  Serial.begin(9600); // Start serial communication for monitoring
  mySerial.begin(9600); // Arduino and Raspberry Pi communication

  // Start by moving to 180 degrees
  motor.write(180,100);
  Serial.println("Homing to 180...");
  currentAngle = 180;
}

void loop() {
  Serial.println("Main");
  motor.write(0);
  motor.write(90);
  while (true){
      motor.stop();
  }

}