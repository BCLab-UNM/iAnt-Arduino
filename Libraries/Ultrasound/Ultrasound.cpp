/* Library for SRF05 Ultrasonic Ranger
*	on Arduino platform
*	Rev 10/23/11
*	Joshua Hecker
*/

#include <Ultrasound.h>

/**
*	Constructor args are pins for trigger, echo, instantions of Compass and Movement classes
*	and last arg for pin for use in single wire mode (see initial comment block)
**/
Ultrasound::Ultrasound(byte usTrigger, byte usEcho, bool simFlag) {
	pinMode(usTrigger, OUTPUT);
	pinMode(usEcho, INPUT);
	_usTrigger = usTrigger;
	_usEcho = usEcho;
	_simFlag = simFlag;
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

	//Return distance (we cap this at 400 cm, the maximum rating for the ultrasound)
	return min(distance,400);
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