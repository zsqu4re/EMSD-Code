#include <AccelStepper.h>
#include <ezButton.h>

// Define pin numbers for the 4 motor wires of each stepper
#define MOTORL_PIN_1 49 //enable
#define MOTORL_PIN_2 51 //DIR
#define MOTORL_PIN_3 53 //PUL

#define MOTORR_PIN_1 42
#define MOTORR_PIN_2 44
#define MOTORR_PIN_3 46


#define MOTORX_PIN_1 48
#define MOTORX_PIN_2 50
#define MOTORX_PIN_3 52

#define MOTORY_PIN_1 43
#define MOTORY_PIN_2 45
#define MOTORY_PIN_3 47

// Define limit switches
ezButton limitSwitch_Upper_Right(23);
ezButton limitSwitch_Upper_Left(24);
ezButton limitSwitch_Lower_Right(27);
ezButton limitSwitch_Lower_Left(22);
ezButton limitSwitch_Y_Back(28);
ezButton limitSwitch_Y_Front(29);
ezButton limitSwitch_X_Left(26);
ezButton limitSwitch_X_Right(25);
// Steps per revolution and distance per revolution for your motors
const int stepsPerRev = 200;
const float mmPerRev = 4.0;

// Initialize two stepper motors using FULL4WIRE configuration
AccelStepper stepperL(AccelStepper::DRIVER, MOTORL_PIN_3, MOTORL_PIN_2);
AccelStepper stepperR(AccelStepper::DRIVER, MOTORR_PIN_3, MOTORR_PIN_2);

// Position structure
struct Position {
  int x;
  int y;
  int z;
};

Position current_position = {0, 0, 0}; // Initialize to origin

// Convert millimeters to steps
int toSteps(float mm) {
  return static_cast<int>(2*mm / mmPerRev * stepsPerRev);
}

// Set speed in RPM for motors
void setSpeedRPM(float rpm) {
  float speedInStepsPerSecond = (rpm * stepsPerRev) / 60.0;
  stepperL.setMaxSpeed(speedInStepsPerSecond);
  stepperR.setMaxSpeed(speedInStepsPerSecond);
}

void shutdown() {
  while (true) {
    // Stay in this loop forever
    Serial.println("stopped");
  }
}

// Homing function
void home() {
  int maxSteps = 10000;  // Define maximum steps for homing
  int defaultRPM = 600;
  int defaultAcc = 6000;

  // Set speed and acceleration
  setSpeedRPM(defaultRPM);
  stepperL.setAcceleration(defaultAcc);
  stepperR.setAcceleration(defaultAcc);

  stepperL.move(-maxSteps);
  stepperR.move(-maxSteps);

  unsigned long startTime = millis();
  const unsigned long maxHomeTime = 10000; // 10 seconds timeout

  while (!(limitSwitch_Lower_Left.isPressed() && limitSwitch_Lower_Right.isPressed())) {
    limitSwitch_Lower_Left.loop();
    limitSwitch_Lower_Right.loop();

    if (millis() - startTime > maxHomeTime) {
      Serial.println("Homing timeout - exiting");
      return;
    }

    if (!limitSwitch_Lower_Left.isPressed()) {
      stepperL.run();
    } else {
      stepperL.stop();
      stepperL.setCurrentPosition(0);  // Set home position
    }

    if (!limitSwitch_Lower_Right.isPressed()) {
      stepperR.run();
    } else {
      stepperR.stop();
      stepperR.setCurrentPosition(0);  // Set home position
    }
  }

  Serial.println("Homing completed");
}

// Move to a specific position
void go_to(Position target) {
  int stepsL = toSteps(target.z - current_position.z);
  int stepsR = toSteps(target.z - current_position.z);

  stepperL.move(stepsL);
  stepperR.move(stepsR);

  while (stepperL.isRunning() || stepperR.isRunning()) {
    stepperL.run();
    stepperR.run();
  }

  current_position = target; // Update current position
}

void setup() {
  Serial.begin(9600);

  int defaultRPM = 600;
  int defaultAcc = 6000;

  setSpeedRPM(defaultRPM);
  stepperL.setAcceleration(defaultAcc);
  stepperR.setAcceleration(defaultAcc);

  limitSwitch_Lower_Left.setDebounceTime(50);
  limitSwitch_Lower_Right.setDebounceTime(50);

  delay(4000);
  // home();  // Perform homing
}

void loop() {
  Position position1 = {0, 0, 600}; // Example target position
  Position position2 = {0, 0, -100};  // Home position

  go_to(position1);  // Move to position1
  delay(2000);

  go_to(position2);  // Return to home position
  delay(2000);

  
  shutdown();
  
}
