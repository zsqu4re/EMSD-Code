#include <AccelStepper.h>
#include <ezButton.h>


#define MOTORL_PIN_1 49 //PUL
#define MOTORL_PIN_2 53 //DIR

#define MOTORR_PIN_1 42 
#define MOTORR_PIN_2 40

#define MOTORX_PIN_1 48
#define MOTORX_PIN_2 36

#define MOTORY_PIN_1 43
#define MOTORY_PIN_2 45

ezButton limitSwitch_Upper_Right(25);
ezButton limitSwitch_Upper_Left(35);
ezButton limitSwitch_Lower_Right(29);
ezButton limitSwitch_Lower_Left(31);
ezButton limitSwitch_Y_Back(23);
ezButton limitSwitch_Y_Front(37);
ezButton limitSwitch_X_Left(33);
ezButton limitSwitch_X_Right(27);

const int microstep = 1;
const int stepsPerRev = 200;
const float mmPerRev = 4.0;
const float Zmax = 1000; //mm 
const float Xmax = 550;
const float Ymax = 300;
const int maxSteps = Zmax * (microstep*stepsPerRev / mmPerRev);  // Convert max distance to steps

AccelStepper stepperL(AccelStepper::DRIVER, MOTORL_PIN_1, MOTORL_PIN_2);
AccelStepper stepperR(AccelStepper::DRIVER, MOTORR_PIN_1, MOTORR_PIN_2);
AccelStepper stepperX(AccelStepper::DRIVER, MOTORX_PIN_1, MOTORX_PIN_2);
AccelStepper stepperY(AccelStepper::DRIVER, MOTORY_PIN_1, MOTORY_PIN_2);

struct Position {
  int x;
  int y;
  int z;
};

Position current_position = {0, 0, 0};

int32_t toSteps_Z(float mm) {
    Serial.println(mm);
    int32_t steps = ((mm / mmPerRev) * stepsPerRev);
    Serial.println(steps);
    return steps;
}

int toSteps_XY(float mm) {
    float OD = 18.57;                       // Pulley outer diameter in mm
    float Pi = 3.14159;                    // Pi value
    float circumference = OD * Pi;         // Pulley circumference in mm                // Microstepping setting
    return static_cast<int32_t>(mm / circumference * stepsPerRev * 2);
}

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

  int stepsX = toSteps_XY(target.x);
  int stepsY = toSteps_XY(target.y);
  int stepsZ = toSteps_Z(target.z);
  Serial.println(stepsZ);
  Serial.println(stepsX);
  Serial.println(stepsY);
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
}

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

  // Move motors to search for limit switches
  stepperL.move(toSteps_Z(-Zmax));
  stepperR.move(toSteps_Z(-Zmax));
  stepperX.move(toSteps_XY(-800 * 25.4));
  stepperY.move(toSteps_XY(-250 * 25.4));

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
  Position position0 = {10,10,-10};
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
  // put your setup code here, to run once:
  Serial.begin(9600);
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

  delay(1000);
  home();
  Position position0 = {10,10,648};
  go_to(position0);
  Serial.println("Start Main");

  
}

void loop() {
  // put your main code here, to run repeatedly:



}


