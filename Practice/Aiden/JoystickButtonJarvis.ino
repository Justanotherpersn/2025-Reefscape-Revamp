#include <Joystick.h>  // Use MHeironimus's Joystick library

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
	16, 0,                  // Button Count, Hat Switch Count
	false, false, false,    // No X, Y, or Z axes
	false, false, false,    // No Rx, Ry, or Rz
	false, false,           // No rudder or throttle
	false, false, false);   // No accelerator, brake, or steering

#include <HID_Buttons.h>  // Must import AFTER Joystick.h


void setup() {
  for (int i = 2; i <= 13; i++) {
	  pinMode(i, INPUT_PULLUP);
  }
  for (int i = 22; i<=25; i++) {
    pinMode(i, INPUT_PULLUP);
  }
	Joystick.begin();
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i <= 11; i++) {
    boolean pressed = !digitalRead(i + 2);
    Joystick.setButton(i, pressed);
  }
  for (int i = 12; i <= 16; i++) {
    boolean pressed = !digitalRead(i+10);
    Joystick.setButton(i, pressed);
  }
}
