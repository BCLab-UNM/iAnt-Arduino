/* Library for SRF05 Ultrasonic Ranger
*	on Arduino platform
*	Rev 10/23/11
*	Joshua Hecker
*/

#include <Ultrasound.h>

/**
*	Constructor args are pins for trigger and echo, a flag denoting simulator mode, and
*	the maximum range of the ultraound
**/
Ultrasound::Ultrasound(byte usTrigger, byte usEcho, bool simFlag, float maxRange) {
	pinMode(usTrigger, OUTPUT);
	pinMode(usEcho, INPUT);
	_usTrigger = usTrigger;
	_usEcho = usEcho;
	_simFlag = simFlag;
	_maxRange = maxRange;
}

/**
*	Checks for any objects at or within user-specified boundry (in cm)
*	Returns false if clear, true otherwise
**/
bool Ultrasound::collisionDetection(float boundry)
{
	if (!_simFlag && (distance() <= boundry)) return true;
	return false;
}

/** 
*	Returns 'float' in CM of distance from ranging device to object
*	See SRF05 doc for information about shape and spread of ultrasonic
*	pattern.  Accurate to 400 cm as per doc.
**/
float Ultrasound::distance()
{
    //50ms delay to ensure no overlap with previous measurements
	delay(50);
    
	//Trigger pulse
	digitalWrite(_usTrigger,HIGH);
	delayMicroseconds(10);
	digitalWrite(_usTrigger,LOW);
	
	//Wait for return pulse and record length of time (measured in microseconds)
	//Note that we use half of the total round-trip time
	unsigned long time = pulseIn(_usEcho,HIGH) / 2;
	
	//Convert time to distance (measured in cm)
	//Assumes speed of sound of 344.83 m/s
	int distance = time * (1/29.0);

	//Return distance, capped at maximum range of ultrasound
	return min(distance,_maxRange);
}

///////////////////
////LEGACY CODE////
///////////////////

/** 
*	Returns 'float' in cm of distance from ranging device to object.
*	This is for use with a single wire tx/rx.
*	See SRF05 doc for information about shape and spread of ultrasonic
*	pattern.  Accurate to 400 cm as per doc.
**/
//  int Ultrasound::swDistance() { 
//    long duration, cm; 
//    pinMode(_us_singleWire, OUTPUT); 
//    digitalWrite(_us_singleWire, LOW); 
//    delayMicroseconds(2); 
//    digitalWrite(_us_singleWire, HIGH); 
//    delayMicroseconds(5); 
//    digitalWrite(_us_singleWire, LOW); 
//    pinMode(_us_singleWire, INPUT); 
//    duration = pulseIn(_us_singleWire, HIGH); // convert the time into a distance 
//     cm = microsecondsToCentimeters(duration); 
//   return cm ; 
// } 


// long Ultrasound::microsecondsToCentimeters(long microseconds) { 
//     return microseconds / 60.0; // The speed of sound is 340 m/s or 29 microseconds per centimeter.
// } 