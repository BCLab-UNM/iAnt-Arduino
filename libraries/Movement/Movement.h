/* 	Library for Ardumoto Motor Driver Shield DEV-09815
	on Arduino platform
	Rev 10/23/11
	Joshua Hecker
*/

#ifndef Movement_h
#define Movement_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
  
#include <Utilities.h>

class Movement
{
	public:
		//Constructors
		Movement(byte rightSpeedPin, byte leftSpeedPin, byte rightDirectionPin, 
				byte leftDirectionPin, bool simFlag);
		
		//Functions
		void forward(byte leftSpeed, byte rightSpeed);
		void backward(byte leftSpeed, byte rightSpeed);
		void rotateRight(byte speed);
		void rotateLeft(byte speed);
		void stopMove();
		
	private:
		byte _rightSpeedPin, _leftSpeedPin, _rightDirectionPin, _leftDirectionPin;
		bool _simFlag;
};

#endif