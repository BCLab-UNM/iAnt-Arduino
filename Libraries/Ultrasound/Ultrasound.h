/* Library for SRF05 Ultrasonic Ranger
*	on Arduino platform
*	Rev 10/23/11
*	Joshua Hecker
*/

#ifndef Ultrasound_h
#define Ultrasound_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Ultrasound {
	public:
		//Constructors
		Ultrasound(byte usTrigger, byte usEcho, bool simFlag, float maxRange);
		
		//Functions
		bool collisionDetection(float boundry);
		float distance();
		
		//Legacy Functions
		//int swDistance();
		//long microsecondsToCentimeters(long microseconds);
	
	private:
		//Variables
		byte _usTrigger, _usEcho;
		bool _simFlag;
		float _maxRange;
};

#endif