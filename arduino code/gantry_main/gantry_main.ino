#include <AccelStepper.h>
#include <ezButton.h>
#include <SoftwareSerial.h>

SoftwareSerial piSerial(10, 11);

// Define pin numbers for the 4 motor wires of each stepper

#define MOTORL_PIN_1 49 //PUL
#define MOTORL_PIN_2 53 //DIR
// #define MOTORL_PIN_3 53 //ENA

// #define MOTORR_PIN_1 48 
// #define MOTORR_PIN_2 50
// #define MOTORR_PIN_3 52

#define MOTORR_PIN_1 42 
#define MOTORR_PIN_2 40
// #define MOTORR_PIN_3 46

#define MOTORX_PIN_1 48
#define MOTORX_PIN_2 36
// #define MOTORX_PIN_3 52

#define MOTORY_PIN_1 43
#define MOTORY_PIN_2 45
// #define MOTORY_PIN_3 47

// Define limit switches
ezButton limitSwitch_Upper_Right(25);
ezButton limitSwitch_Upper_Left(35);
ezButton limitSwitch_Lower_Right(29);
ezButton limitSwitch_Lower_Left(31);
ezButton limitSwitch_Y_Back(23);
ezButton limitSwitch_Y_Front(37);
ezButton limitSwitch_X_Left(33);
ezButton limitSwitch_X_Right(27);


// Steps per revolution and distance per revolution for your motors
const int microstep = 1;
const int stepsPerRev = 200;
const float mmPerRev = 4.0;
const float Zmax = 1000; //mm 
const float Xmax = 550;
const float Ymax = 300;
const int maxSteps = Zmax * (microstep*stepsPerRev / mmPerRev);  // Convert max distance to steps

// Initialize four stepper motors using FULL4WIRE configuration
// AccelStepper stepperL(AccelStepper::FULL4WIRE, MOTORL_PIN_1, MOTORL_PIN_2, MOTORL_PIN_3, MOTORL_PIN_4);
// AccelStepper stepperR(AccelStepper::FULL4WIRE, MOTORR_PIN_1, MOTORR_PIN_2, MOTORR_PIN_3, MOTORR_PIN_4);
// AccelStepper stepperL(AccelStepper::FULL3WIRE, MOTORL_PIN_1, MOTORL_PIN_2, MOTORL_PIN_3);
// AccelStepper stepperR(AccelStepper::FULL3WIRE, MOTORR_PIN_1, MOTORR_PIN_2, MOTORR_PIN_3);

AccelStepper stepperL(AccelStepper::DRIVER, MOTORL_PIN_1, MOTORL_PIN_2);
AccelStepper stepperR(AccelStepper::DRIVER, MOTORR_PIN_1, MOTORR_PIN_2);
// AccelStepper stepperX(AccelStepper::FULL3WIRE, MOTORX_PIN_1, MOTORX_PIN_2, MOTORX_PIN_3);
// AccelStepper stepperY(AccelStepper::FULL3WIRE, MOTORY_PIN_1, MOTORY_PIN_2, MOTORY_PIN_3);
AccelStepper stepperX(AccelStepper::DRIVER, MOTORX_PIN_1, MOTORX_PIN_2);
AccelStepper stepperY(AccelStepper::DRIVER, MOTORY_PIN_1, MOTORY_PIN_2);

// Structure to hold position records
struct Position {
  int x;
  int y;
  int z;
};

// Global variable to track the current position
Position current_position = {0, 0, 0}; // Initialize to origin (0, 0, 0)


int32_t toSteps_Z(float mm) {
    // Serial.println(mm);
    int32_t steps = ((-mm / mmPerRev) * stepsPerRev);
    // Serial.println(steps);
    return steps;
}

int32_t toSteps_XY(float mm) {
    const float OD = 18.57;                       // Pulley outer diameter in mm
    const float Pi = 3.14159;                    // Pi value
    const float circumference = OD * Pi;         // Pulley circumference in mm                // Microstepping setting
    return static_cast<int32_t>( mm / circumference * stepsPerRev * 2);
}

// int32_t toSteps_Y(float mm) {
//     const float OD = 18.57;                       // Pulley outer diameter in mm
//     const float Pi = 3.14159;                    // Pi value
//     const float circumference = OD * Pi;         // Pulley circumference in mm                // Microstepping setting
//     return static_cast<int32_t>(mm / circumference * stepsPerRev * 2);
// }

// Function to set speed in RPM for all motors
void setSpeedRPM_Z(float rpm) {
  float speedInStepsPerSecond = (rpm * stepsPerRev) / 60.0;
  stepperL.setMaxSpeed(speedInStepsPerSecond);
  stepperR.setMaxSpeed(speedInStepsPerSecond);
}

void setSpeedRPM_X(float rpm) {
  float speedInStepsPerSecond = (rpm * stepsPerRev) / 60.0;
  stepperX.setMaxSpeed(speedInStepsPerSecond);

}

void setSpeedRPM_Y(float rpm) {
  float speedInStepsPerSecond = (rpm * stepsPerRev) / 60.0;
  stepperY.setMaxSpeed(speedInStepsPerSecond);
}


bool checkSwitches() {
  limitSwitch_Upper_Right.loop();
  limitSwitch_Upper_Left.loop();
  limitSwitch_Lower_Right.loop();
  limitSwitch_Lower_Left.loop();
  limitSwitch_X_Left.loop();
  limitSwitch_X_Right.loop();
  limitSwitch_Y_Back.loop();
  limitSwitch_Y_Front.loop();

  bool isAnySwitchPressed = false;

  if (limitSwitch_Upper_Right.isPressed()) {
    Serial.println("Upper Right switch is pressed");
    isAnySwitchPressed = true;
  }
  if (limitSwitch_Upper_Left.isPressed()) {
    Serial.println("Upper Left switch is pressed");
    isAnySwitchPressed = true;
  }
  if (limitSwitch_Lower_Right.isPressed()) {
    Serial.println("Lower Right switch is pressed");
    isAnySwitchPressed = true;
  }
  if (limitSwitch_Lower_Left.isPressed()) {
    Serial.println("Lower Left switch is pressed");
    isAnySwitchPressed = true;
  }
  if (limitSwitch_X_Left.isPressed()) {
    Serial.println("X Left switch is pressed");
    isAnySwitchPressed = true;
  }
  if (limitSwitch_X_Right.isPressed()) {
    Serial.println("X Right switch is pressed");
    isAnySwitchPressed = true;
  }
  if (limitSwitch_Y_Back.isPressed()) {
    Serial.println("Y Back switch is pressed");
    isAnySwitchPressed = true;
  }
  if (limitSwitch_Y_Front.isPressed()) {
    Serial.println("Y Front switch is pressed");
    isAnySwitchPressed = true;
  }

  return isAnySwitchPressed;
}


void shutdown() {
  Serial.println("stopped");
  while (true) {
    // Stay in this loop forever
    
  }
}

void go_to(Position target) {
  int32_t stepsX = toSteps_XY(target.x);
  int32_t stepsY = toSteps_XY(target.y);
  int32_t stepsZ = -toSteps_Z(target.z);
  // Move each motor to its target position

  Serial.println(stepsZ);
  Serial.println(stepsX);
  Serial.println(stepsY);

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
  Serial.println("Complete");
}


// Home both motors using limit switches
void home(int defaultZRPM = 600, int defaultXYRPM = 200, int defaultZAcc = 5000, int defaultXYAcc = 1000) {
  Serial.println("Starting homing sequence...");

  // Set speed and acceleration for all motors
  setSpeedRPM_Z(defaultZRPM);    // Z motors
  setSpeedRPM_X(defaultXYRPM); // X and Y slower for precision
  setSpeedRPM_Y(defaultXYRPM);
  stepperL.setAcceleration(defaultZAcc);
  stepperR.setAcceleration(defaultZAcc);
  stepperX.setAcceleration(defaultXYAcc);
  stepperY.setAcceleration(defaultXYAcc);

  // Initialize homing flags
  bool stepperL_Homed = false, stepperR_Homed = false;
  bool stepperX_Homed = false, stepperY_Homed = false;

  int stepsX = toSteps_XY((-800 * 25.4));
  int stepsY = toSteps_XY((-800 * 25.4));
  int stepsZ = toSteps_Z(-Zmax);
  // Move each motor to its target position
  stepperX.moveTo(stepsX);
  stepperY.moveTo(stepsY);
  stepperL.moveTo(stepsZ);
  stepperR.moveTo(stepsZ);

  // Move motors to search for limit switches
  // stepperL.move(toSteps_Z(-Zmax));
  // stepperR.move(toSteps_Z(-Zmax));
  // stepperX.move(toSteps_XY(-800 * 25.4));
  // stepperY.move(toSteps_XY(-250 * 25.4));

  while (!stepperL_Homed || !stepperR_Homed || !stepperX_Homed || !stepperY_Homed) {
    // Update all limit switch states
    limitSwitch_Lower_Left.loop();
    limitSwitch_Lower_Right.loop();
    limitSwitch_X_Left.loop();
    limitSwitch_Y_Front.loop();

    // Homing each motor
    if (!stepperL_Homed && limitSwitch_Lower_Left.isPressed()) {
      stepperL.stop();
      stepperL.setCurrentPosition(0);
      stepperL_Homed = true;
      Serial.println("StepperL homed.");
    } else if (!stepperL_Homed) {
      stepperL.run();
    }

    if (!stepperR_Homed && limitSwitch_Lower_Right.isPressed()) {
      stepperR.stop();
      stepperR.setCurrentPosition(0);
      stepperR_Homed = true;
      Serial.println("StepperR homed.");
    } else if (!stepperR_Homed) {
      stepperR.run();
    }

    if (!stepperX_Homed && limitSwitch_X_Left.isPressed()) {
      stepperX.stop();
      stepperX.setCurrentPosition(0);
      stepperX_Homed = true;
      Serial.println("StepperX homed.");
    } else if (!stepperX_Homed) {
      stepperX.run();
    }

    if (!stepperY_Homed && limitSwitch_Y_Front.isPressed()) {
      stepperY.stop();
      stepperY.setCurrentPosition(0);
      stepperY_Homed = true;
      Serial.println("StepperY homed.");
    } else if (!stepperY_Homed) {
      stepperY.run();
    }
  }

  delay(500);
  Position position0 = {10,10,10};
  go_to(position0);

  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  Serial.println("Homing sequence completed.");

  // Clear all limit switch states to prevent carry-over
  delay(500); // Allow stabilization
  limitSwitch_Lower_Left.loop();
  limitSwitch_Lower_Right.loop();
  limitSwitch_X_Left.loop();
  limitSwitch_X_Right.loop();
  limitSwitch_Y_Back.loop();
  limitSwitch_Y_Front.loop();
  limitSwitch_Upper_Left.loop();
  limitSwitch_Upper_Right.loop();

  // Ensure all switches are reset
  Serial.println("Switch states reset after homing.");
}



void setup() {
  Serial.begin(9600);
  // piSerial.begin(9600);  // Initialize serial communication at 9600 baud rate

  int defaultZRPM = 600;
  int defaultXRPM = 200;
  int defaultYRPM = 100;
  int defaultZAcc = 5000; 
  int defaultXYAcc = 3000;

  setSpeedRPM_Z(defaultZRPM);    // Z motors
  setSpeedRPM_X(defaultXRPM); // X and Y slower for precision
  setSpeedRPM_Y(defaultYRPM);
  stepperL.setAcceleration(defaultZAcc);
  stepperR.setAcceleration(defaultZAcc);
  stepperX.setAcceleration(defaultXYAcc);
  stepperY.setAcceleration(defaultXYAcc);


  limitSwitch_Upper_Right.setDebounceTime(50);
  limitSwitch_Upper_Left.setDebounceTime(50);
  limitSwitch_Lower_Right.setDebounceTime(50);
  limitSwitch_Lower_Left.setDebounceTime(50);
  limitSwitch_X_Left.setDebounceTime(50);
  limitSwitch_X_Right.setDebounceTime(50);
  limitSwitch_Y_Back.setDebounceTime(50);
  limitSwitch_Y_Front.setDebounceTime(50);

  // Home the motors at startup
  delay(1000);
  home();
  Serial.println("Start Main");
  Serial.flush();
}

void loop(){
  // Position position1 = {0,0,0};  // Target position (5 mm, 5 mm, 60 mm)
  Position position2 = {200,100,684};  // Target position (5 mm, 5 mm, 60 mm)
  Position position3 = {10,10,342};   // Origin position (0 mm, 0 mm, 0 mm)
  Position position4 = {400,50,400};   // Origin position (0 mm, 0 mm, 0 mm)
  Position position5 = {400,200,400};   // Origin position (0 mm, 0 mm, 0 mm)
  Position position6 = {500,50,500};   // Origin position (0 mm, 0 mm, 0 mm)

  go_to(position2);  // Move to position1
  delay(1000);

  go_to(position3);  // Move back to origin
  delay(1000);

  // go_to(position3);  // Move back to origin
  // delay(1000);

  // go_to(position4);  // Move back to origin
  // delay(1000);

  // go_to(position5);  // Move back to origin
  // delay(1000);

  // go_to(position6);  // Move back to origin
  // delay(1000);

  shutdown();
}