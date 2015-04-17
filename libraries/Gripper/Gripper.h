/* 	Gripper Library for Ardumoto Motor Driver Shield DEV-09815
	on Arduino platform
	Rev 4/17/15
	Matthew Fricke
*/

// This class contains an api for controlling the gripper from the IOS level.
#ifndef Gripper_h
#define Gripper_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>

class Gripper
{
public:

  Gripper(int, int, int, int, int, boolean );

  void pickup();

  void putdown();

private:

  Servo pneumatic_servo;
  int pneumatic_servo_pin;
  int pneumatic_servo_pos;
  int pneumatic_servo_min;
  int pneumatic_servo_max;
  int solenoid_pin;
  boolean solenoid_on;

};

#endif

