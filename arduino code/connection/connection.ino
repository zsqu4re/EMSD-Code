void setup() {
  Serial.begin(9600);
  while (!Serial) {  // Wait for serial port to connect if needed
    delay(10);
  }
  Serial.println("Arduino is ready");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    Serial.println("Complete");
    // Serial.print("Received: ");
    // Serial.println(input);
    
    // You can parse and respond here
  }
}
