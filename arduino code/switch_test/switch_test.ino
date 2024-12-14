/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-limit-switch
 */

#include <ezButton.h>

ezButton limitSwitch_Upper_Right(25);
ezButton limitSwitch_Upper_Left(35);
ezButton limitSwitch_Lower_Right(29);
ezButton limitSwitch_Lower_Left(31);
ezButton limitSwitch_Y_Back(23);
ezButton limitSwitch_Y_Front(37);
ezButton limitSwitch_X_Left(33);
ezButton limitSwitch_X_Right(27);  // create ezButton object that attach to pin 7;

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


void setup() {
  Serial.begin(9600);
  limitSwitch_Upper_Right.setDebounceTime(50);
  limitSwitch_Upper_Left.setDebounceTime(50);
  limitSwitch_Lower_Right.setDebounceTime(50);
  limitSwitch_Lower_Left.setDebounceTime(50);
  limitSwitch_X_Left.setDebounceTime(50);
  limitSwitch_X_Right.setDebounceTime(50);
  limitSwitch_Y_Back.setDebounceTime(50);
  limitSwitch_Y_Front.setDebounceTime(50); // set debounce time to 50 milliseconds
}

void loop() {
  // limitSwitch.loop(); // MUST call the loop() function first

  // if(limitSwitch.isPressed() || )
  //   Serial.println("The limit switch: UNTOUCHED -> TOUCHED");

  // if(limitSwitch.isReleased())
  //   Serial.println("The limit switch: TOUCHED -> UNTOUCHED");

  // int state = limitSwitch.getState();
  // if(state == HIGH)
  //   Serial.println("The limit switch: UNTOUCHED");
  // else
  //   Serial.println("The limit switch: TOUCHED");

  checkSwitches();
}
