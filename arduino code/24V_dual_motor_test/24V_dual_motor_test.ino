#include <AccelStepper.h>

// Define the stepper motor and pins
#define MOTORL_PUL_PIN 3  // Pulse (Step) pin
#define MOTORL_DIR_PIN 4  // Direction pin

#define MOTORR_PUL_PIN 10  // Pulse (Step) pin
#define MOTORR_DIR_PIN 11 // Direction pin


// Initialize AccelStepper in DRIVER mode
AccelStepper stepperR(AccelStepper::DRIVER, MOTORL_PUL_PIN, MOTORL_DIR_PIN);
AccelStepper stepperL(AccelStepper::DRIVER, MOTORR_PUL_PIN, MOTORR_DIR_PIN);

const int stepsPerRev = 200; // Steps per revolution
const float mmPerRev = 4.0;  // mm per revolution
const int step_pulse = 4;    // Multiplier for pulse steps

// Convert distance in mm to steps
int toSteps(float mm) {
    return static_cast<int>(mm / mmPerRev * stepsPerRev);
}

// Function to move both steppers synchronously
void moveZ(float mm) {
    int steps = toSteps(mm); // Convert mm to steps
    stepperL.move(steps * step_pulse); // Move Left motor
    stepperR.move(steps * step_pulse); // Move Right motor

    // Run both steppers until the motion is complete
    while (stepperL.isRunning() || stepperR.isRunning()) {
        stepperL.run();
        stepperR.run();

    }
}

void setup() {
    // Set maximum speed and acceleration for both motors
    Serial.begin(9600);
    // stepperR.setMaxSpeed(4000);  // Maximum speed in steps/second
    stepperL.setMaxSpeed(12000);

    stepperR.setMaxSpeed(12000); // Set a limit greater than required speed

  // Set the desired speed (in steps per second)
    stepperR.setSpeed(12000); // Speed for 200 RPM

    stepperR.setAcceleration(80000); // Acceleration in steps/second^2
    stepperL.setAcceleration(80000);
}

void loop() {




    moveZ(80);
    delay(1000); // Wait for a second
    // Move back 8mm in the negative direction
    moveZ(-80);

    delay(1000); // Wait for a second
}
