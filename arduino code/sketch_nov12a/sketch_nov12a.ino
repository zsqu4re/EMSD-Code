#include <Stepper.h>
#include <ezButton.h>

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 2,3,4,5);

ezButton limitSwitchUpperRight(8);
ezButton limitSwitchUpperLeft(9);
ezButton limitSwitchLowerRight(6);
ezButton limitSwitchLowerLeft(7);

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(200);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  limitSwitchLowerLeft.loop();
  limitSwitchLowerRight.loop();
  // step one revolution in one direction:
  if(limitSwitchLowerLeft.isReleased()){
  Serial.println("clockwise");
  myStepper.step(5*stepsPerRevolution);
  delay(1000);
  
  // step one revolution in the other direction:
  Serial.println("counterclockwise");
  myStepper.step(-5*stepsPerRevolution);
  delay(1000);
}
else{
  myStepper.setSpeed(0);
}
}