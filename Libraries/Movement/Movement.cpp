/* Library for Ardumoto Motor Driver Shield DEV-09815
*	on Arduino platform
*	Rev 10/23/11
*	Joshua Hecker
*/

#include <Movement.h>

/**
*	Constructor args are pins for ArduMoto shield (3 is permanent) and Compass instantiation
**/
Movement::Movement(byte rightSpeedPin, byte leftSpeedPin, byte rightDirectionPin, 
					byte leftDirectionPin, bool simFlag) {
	pinMode(rightSpeedPin, OUTPUT);
	pinMode(leftSpeedPin, OUTPUT);
	pinMode(rightDirectionPin, OUTPUT);
	pinMode(leftDirectionPin, OUTPUT);
	_rightSpeedPin = rightSpeedPin;
	_leftSpeedPin = leftSpeedPin;
	_rightDirectionPin = rightDirectionPin;
	_leftDirectionPin = leftDirectionPin;
	_simFlag = simFlag;
}

/**
*	Sets forward motion at speed, args are for left and right tread speed
**/
void Movement::forward(byte leftSpeed, byte rightSpeed) {
	if (!_simFlag) {
		digitalWrite(_leftDirectionPin,LOW);
		digitalWrite(_rightDirectionPin,LOW);
		analogWrite(_leftSpeedPin,leftSpeed);
		analogWrite(_rightSpeedPin,rightSpeed);
	}
}

/**
*	Sets backward motion at speed, args are for left and right tread speed
**/
void Movement::backward(byte leftSpeed, byte rightSpeed) {
	if (!_simFlag) {
		digitalWrite(_leftDirectionPin,HIGH);
		digitalWrite(_rightDirectionPin,HIGH);
		analogWrite(_leftSpeedPin,leftSpeed);
		analogWrite(_rightSpeedPin,rightSpeed);
	}
}

/**
*	Rotates right at speed s.  Treads will move in opposite directions
**/
void Movement::rotateRight(byte speed) {
	digitalWrite(_rightDirectionPin,HIGH);
	digitalWrite(_leftDirectionPin,LOW);
	analogWrite(_rightSpeedPin,speed);
	analogWrite(_leftSpeedPin,speed);
}

/**
*	Rotates left at speed s.  Treads will move in opposite directions
**/
void Movement::rotateLeft(byte speed) {
	digitalWrite(_rightDirectionPin,LOW);
	digitalWrite(_leftDirectionPin,HIGH);
	analogWrite(_rightSpeedPin,speed);
	analogWrite(_leftSpeedPin,speed);
}

/**
*	Stops all motor motion
**/
void Movement::stopMove() {
	analogWrite(_rightSpeedPin,0);
	analogWrite(_leftSpeedPin,0);
}