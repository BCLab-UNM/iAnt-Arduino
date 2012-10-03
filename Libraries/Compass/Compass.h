/* Library for Honeywell 5263 Compass Functions
*	on Arduino platform
*	Rev 12/09/11
*	Joshua Hecker
*/

#ifndef Compass_h
#define Compass_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
  
#include <Wire.h>
#include <Utilities.h>

class Compass
{
	public:
		//Constructors
		Compass();
		Compass(Utilities &ut);
		
		//Functions
		void calibrate();
		float heading();
		void calibrate_start();
		void calibrate_end();
		
		//Legacy Functions
 		//float avgHeading(int s);
 		//byte readEEProm(int s);
 		//void writeEEProm(int s, byte promDat);
 		//byte readRAM();

	private:
		byte _led_grn;
		Utilities *util;
};

#endif