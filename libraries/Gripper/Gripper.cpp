/* Gripper Library for Ardumoto Motor Driver Shield DEV-09815
*	on Arduino platform
*	Rev 4/17/15
*       Matthew Fricke & Jake Nichol
*/
#include <Gripper.h>

Gripper::Gripper( int ps_pin, int ps_pos, int ps_min, int ps_max, int sol_pin, boolean sol_on)
{
  pneumatic_servo_pin = ps_pin;
  pneumatic_servo_pos = ps_pos;
  pneumatic_servo_min = ps_min;
  pneumatic_servo_max = ps_max;
  solenoid_pin = sol_pin;
  solenoid_on = sol_on;

  pinMode(solenoid_pin, OUTPUT);
  pneumatic_servo.attach(pneumatic_servo_pin);
}

void Gripper::pickup()
{
  digitalWrite(solenoid_pin, HIGH);
  pneumatic_servo.write(pneumatic_servo_max);
}

void Gripper::putdown()
{
    pneumatic_servo.write(pneumatic_servo_min);
    digitalWrite(solenoid_pin, LOW);
}
