#include <AccelStepper.h>
#include <ezButton.h>

// Define pin numbers for the 4 motor wires of each stepper

#define MOTORL_PIN_1 46
#define MOTORL_PIN_2 48
#define MOTORL_PIN_3 50
#define MOTORL_PIN_4 52

#define MOTORR_PIN_1 47
#define MOTORR_PIN_2 49
#define MOTORR_PIN_3 51
#define MOTORR_PIN_4 53

#define MOTORX_PIN_1 38
#define MOTORX_PIN_2 40
#define MOTORX_PIN_3 42
#define MOTORX_PIN_4 44

#define MOTORY_PIN_1 39
#define MOTORY_PIN_2 41
#define MOTORY_PIN_3 43
#define MOTORY_PIN_4 45

// Define limit switches
ezButton limitSwitch_Upper_Right(23);
ezButton limitSwitch_Upper_Left(24);
ezButton limitSwitch_Lower_Right(27);
ezButton limitSwitch_Lower_Left(22);
ezButton limitSwitch_Y_Left(11);
ezButton limitSwitch_Y_Right(12);
ezButton limitSwitch_X_Left(26);
ezButton limitSwitch_X_Right(25);


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

Position positions[100]; // Array to store up to 100 positions
int positionIndex = 0;   // Index to track the current position in the array

int toSteps(float mm) {
    return static_cast<int>(mm / mmPerRev * stepsPerRev/2);
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
void shutdown() {
  while (true) {
    // Stay in this loop forever
    Serial.println("stopped");
  }
}

// Home both motors using limit switches
void home() {
  Serial.println("Starting homing sequence...");

  int defaultRPM = 300;
  int defaultAcc = 3000;

  // Set speed and acceleration for all motors
  setSpeedRPM(defaultRPM);
  stepperL.setAcceleration(defaultAcc);
  stepperR.setAcceleration(defaultAcc);
  stepperX.setAcceleration(defaultAcc);
  stepperY.setAcceleration(defaultAcc);

  // Initialize homing flags
  bool stepperL_Homed = false;
  bool stepperR_Homed = false;
  bool stepperX_Homed = false;
  bool stepperY_Homed = true;

  // Set motors to move towards their respective limit switches
  stepperL.move(-maxSteps);
  stepperR.move(-maxSteps);
  stepperX.move(-toSteps(800 * 25.4));
  stepperY.move(-toSteps(250 * 25.4));

  while (!stepperL_Homed || !stepperR_Homed || !stepperX_Homed || !stepperY_Homed) {
    // Update limit switch states
    limitSwitch_Lower_Left.loop();
    limitSwitch_Lower_Right.loop();
    limitSwitch_X_Left.loop();
    limitSwitch_Y_Right.loop();

    // StepperL homing
    if (!stepperL_Homed) {
      if (limitSwitch_Lower_Left.isPressed()) {
        stepperL.stop();
        stepperL.setCurrentPosition(0); // Set home position
        stepperL_Homed = true;
        Serial.println("#######################################");
        Serial.println("StepperL homed.");
        Serial.println("#######################################");
      } else {
        stepperL.run();
      }
    }

    // StepperR homing
    if (!stepperR_Homed) {
      if (limitSwitch_Lower_Right.isPressed()) {
        stepperR.stop();
        stepperR.setCurrentPosition(0); // Set home position
        stepperR_Homed = true;
        Serial.println("#######################################");
        Serial.println("StepperR homed.");
        Serial.println("#######################################");
      } else {
        stepperR.run();
      }
    }

    // StepperX homing
    if (!stepperX_Homed) {
      if (limitSwitch_X_Left.isPressed()) {
        stepperX.stop();
        stepperX.setCurrentPosition(0); // Set home position
        stepperX_Homed = true;
        Serial.println("#######################################");
        Serial.println("StepperX homed.");
        Serial.println("#######################################");
      } else {
        stepperX.run();
      }
    }

    // StepperY homing
    if (!stepperY_Homed) {
      if (limitSwitch_Y_Right.isPressed()) {
        stepperY.stop();
        stepperY.setCurrentPosition(0); // Set home position
        stepperY_Homed = true;
        Serial.println("#######################################");
        Serial.println("StepperY homed.");
        Serial.println("#######################################");
      } else {
        stepperY.run();
      }
    }
  }

  Serial.println("Homing sequence completed.");
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

  // Record the position if no switches were triggered
  if (positionIndex < 100) { 
    positions[positionIndex] = target;
    positionIndex++;
    Serial.print("Position recorded: ");
    Serial.print("X=");
    Serial.print(target.x);
    Serial.print(", Y=");
    Serial.print(target.y);
    Serial.print(", Z=");
    Serial.println(target.z);
  } else {
    Serial.println("Position storage full.");
  }

  current_position = target; // Update the global current_position
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
  delay(4000);
  home();
}

void loop(){
  Position position1 = {-200, 0, 200};  // Target position (5 mm, 5 mm, 60 mm)
  Position position2 = {-100, 0, 100};  // Target position (5 mm, 5 mm, 60 mm)
  Position position3 = {0, 0, 20};   // Origin position (0 mm, 0 mm, 0 mm)

  go_to(position1);  // Move to position1
  delay(2000);

  go_to(position2);  // Move back to origin
  delay(2000);

  // go_to(position3);  // Move back to origin
  // delay(2000);

  shutdown();
}
