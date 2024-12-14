#include <AccelStepper.h>
#include <ezButton.h>

// Define pin numbers for the 4 motor wires of each stepper

#define MOTORL_PIN_1 47
#define MOTORL_PIN_2 49
#define MOTORL_PIN_3 51
#define MOTORL_PIN_4 53

#define MOTORR_PIN_1 37
#define MOTORR_PIN_2 39
#define MOTORR_PIN_3 41
#define MOTORR_PIN_4 43

#define MOTORX_PIN_1 1
#define MOTORX_PIN_2 1
#define MOTORX_PIN_3 1
#define MOTORX_PIN_4 1

#define MOTORY_PIN_1 46
#define MOTORY_PIN_2 48
#define MOTORY_PIN_3 50
#define MOTORY_PIN_4 52

// Define limit switches
ezButton limitSwitch_Upper_Right(29);
ezButton limitSwitch_Upper_Left(23);
ezButton limitSwitch_Lower_Right(24);
ezButton limitSwitch_Lower_Left(22);
ezButton limitSwitch_X_Left(25);
ezButton limitSwitch_X_Right(28);
ezButton limitSwitch_Y_Left(27);
ezButton limitSwitch_Y_Right(26);

// Steps per revolution and distance per revolution for your motors
const int stepsPerRev = 200;
const float mmPerRev = 4.0;
const int maxSteps = 31 * 25.4 * (stepsPerRev / mmPerRev);  // Convert max distance to steps

// Initialize four stepper motors using FULL4WIRE configuration
AccelStepper stepperL(AccelStepper::FULL4WIRE, MOTORL_PIN_1, MOTORL_PIN_2, MOTORL_PIN_3, MOTORL_PIN_4);
AccelStepper stepperR(AccelStepper::FULL4WIRE, MOTORR_PIN_1, MOTORR_PIN_2, MOTORR_PIN_3, MOTORR_PIN_4);
AccelStepper stepperX(AccelStepper::FULL4WIRE, MOTORX_PIN_1, MOTORX_PIN_2, MOTORX_PIN_3, MOTORX_PIN_4);
AccelStepper stepperY(AccelStepper::FULL4WIRE, MOTORY_PIN_1, MOTORY_PIN_2, MOTORY_PIN_3, MOTORY_PIN_4);

// Structure to hold position records
struct Position {
  int x;
  int y;
  int z;
};

// Global variable to track the current position
Position current_position = {0, 0, 0}; // Initialize to origin (0, 0, 0)

int toSteps(float mm) {
    return static_cast<int>(mm / mmPerRev * stepsPerRev);
}

// Function to set speed in RPM for all motors
void setSpeedRPM(float rpm) {
  float speedInStepsPerSecond = (rpm * stepsPerRev) / 60.0;
  stepperL.setMaxSpeed(speedInStepsPerSecond);
  stepperR.setMaxSpeed(speedInStepsPerSecond);
  stepperX.setMaxSpeed(speedInStepsPerSecond);
  stepperY.setMaxSpeed(speedInStepsPerSecond);
}

bool checkSwitches(){
  limitSwitch_Upper_Right.loop();
  limitSwitch_Upper_Left.loop();
  limitSwitch_Lower_Right.loop();
  limitSwitch_Lower_Left.loop();
  limitSwitch_X_Left.loop();
  limitSwitch_X_Right.loop();
  limitSwitch_Y_Left.loop();
  limitSwitch_Y_Right.loop();

  return (
    limitSwitch_Upper_Right.isPressed() ||
    limitSwitch_Upper_Left.isPressed() ||
    limitSwitch_Lower_Right.isPressed() ||
    limitSwitch_Lower_Left.isPressed() ||
    limitSwitch_X_Left.isPressed() ||
    limitSwitch_X_Right.isPressed() ||
    limitSwitch_Y_Left.isPressed() ||
    limitSwitch_Y_Right.isPressed()
  );
}

// Home both motors using limit switches
void home() {
  int defaultRPM = 200;
  int defaultAcc = 3000;
  
  // Set speed and acceleration for all motors
  setSpeedRPM(defaultRPM);
  stepperL.setAcceleration(defaultAcc);
  stepperR.setAcceleration(defaultAcc);
  
  // Move motors towards the limit switches
  stepperL.move(-maxSteps);
  stepperR.move(-maxSteps);

  unsigned long startTime = millis();
  const unsigned long maxHomeTime = 15000; // Max homing time in milliseconds

  // Keep running the motors until both limit switches are triggered or timeout
  while (!(limitSwitch_Lower_Left.isPressed() && limitSwitch_Lower_Right.isPressed())) {
    Serial.println("Homing");
    limitSwitch_Lower_Left.loop();
    limitSwitch_Lower_Right.loop();

    if (millis() - startTime > maxHomeTime) {
      Serial.println("Homing timeout - exiting homing function");
      return;
    }

    if (!limitSwitch_Lower_Left.isPressed()) {
      stepperL.run();  // Keep running left motor towards home
    } else {
      stepperL.stop();  // Stop left motor if home position is reached
      stepperL.setCurrentPosition(0);  // Set left motor home position to 0
    }

    if (!limitSwitch_Lower_Right.isPressed()) {
      stepperR.run();  // Keep running right motor towards home
    } else {
      stepperR.stop();  // Stop right motor if home position is reached
      stepperR.setCurrentPosition(0);  // Set right motor home position to 0
    }
  }

  Serial.println("Homing completed");
}

void go_to(Position target) {
  int stepsX = toSteps(target.x);
  int stepsY = toSteps(target.y);
  int stepsZ = toSteps(target.z);

  // Move each motor to its target position
  stepperX.moveTo(stepsX);
  stepperY.moveTo(stepsY);
  stepperL.moveTo(stepsZ);
  stepperR.moveTo(stepsZ);

  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0 || stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    if (checkSwitches()) {
      Serial.println("Limit switch activated - stopping all motors.");
      stepperX.stop();
      stepperY.stop();
      stepperL.stop();
      stepperR.stop();
      return; // Exit if a switch is pressed
    }

    stepperX.run();
    stepperY.run();
    stepperL.run();
    stepperR.run();
  }

  // Update the global current_position to the new target position
  current_position = target;
}

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud rate

  int defaultRPM = 200;
  int defaultAcc = 3000;

  setSpeedRPM(defaultRPM);
  stepperL.setAcceleration(defaultAcc);
  stepperR.setAcceleration(defaultAcc);
  stepperX.setAcceleration(defaultAcc);
  stepperY.setAcceleration(defaultAcc);

  limitSwitch_Upper_Right.setDebounceTime(50);
  limitSwitch_Upper_Left.setDebounceTime(50);
  limitSwitch_Lower_Right.setDebounceTime(50);
  limitSwitch_Lower_Left.setDebounceTime(50);
  limitSwitch_X_Left.setDebounceTime(50);
  limitSwitch_X_Right.setDebounceTime(50);
  limitSwitch_Y_Left.setDebounceTime(50);
  limitSwitch_Y_Right.setDebounceTime(50);

  // Home the motors at startup
  home();
}
void loop(){
  Position position1 = {5, 0, 20};  // Target position (5 mm, 5 mm, 60 mm)
  Position position2 = {5, 0, 20};  // Target position (5 mm, 5 mm, 60 mm)
  Position position3 = {0, 0, 0};   // Origin position (0 mm, 0 mm, 0 mm)

  go_to(position1);  // Move to position1
  delay(2000);

  go_to(position2);  // Move back to origin
  delay(2000);
}
