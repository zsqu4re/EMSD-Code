#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX pins for software serial

struct Position {
  int x;
  int y;
  int z;
};

void setup() {
  Serial.begin(9600);   // Debug communication with the PC
  mySerial.begin(9600); // Communication with the Raspberry Pi
  Serial.println("Arduino ready.");
}

void loop() {
  if (mySerial.available() > 0) {
    String input = mySerial.readStringUntil('\n'); // Read from software serial
    input.trim(); // Remove any leading or trailing whitespace
    Position targetPosition;

    if (sscanf(input.c_str(), "%d,%d,%d", &targetPosition.x, &targetPosition.y, &targetPosition.z) == 3) {
      // Parse successful, print to debug and acknowledge via software serial
      Serial.print("Moving to Position: X=");
      Serial.print(targetPosition.x);
      Serial.print(", Y=");
      Serial.print(targetPosition.y);
      Serial.print(", Z=");
      Serial.println(targetPosition.z);

      mySerial.print("Acknowledged: ");
      mySerial.println(input);

      // go_to(targetPosition); // Implement movement logic here
    } else {
      // Invalid format
      Serial.println("Invalid command format. Expected format: 'X,Y,Z'");
      mySerial.println("Error: Invalid format. Use 'X,Y,Z'");
    }
  } else {
    delay(500); // Reduce unnecessary "waiting" logs
  }
}