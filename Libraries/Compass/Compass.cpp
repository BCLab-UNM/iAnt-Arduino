/** Library for Honeywell 5263 Compass Functions
*	on Arduino platform
*	Rev 10/23/11
*	Joshua Hecker
**/

#include <Compass.h>

byte com_slave = 0x42;
const byte com_read = 0x41; 
const byte com_calibration = 0x43; 
const byte com_exit = 0x45;

/**
*	Empty Constructor
**/
Compass::Compass(){}

/**
*	Constructor takes int arg of led pin used for calibration routine indicator
**/
Compass::Compass(Utilities &ut)
{
	util = &ut;
	com_slave = com_slave >> 1; //Shift slave address to the right by 1 bit
  	Wire.begin();
}

/**
*	Calibration Routine
*	LED2 on GPS Shield will light after 15 seconds, when calibration routine begins.
*	Rotate robot smoothly and on level surface through two full rotations over 40 seconds.
*	LED will go out when routine is over.
*	Place delay at end of function call to prevent Arduino from looping and re-entering
*	calibration routine (which might cause failure or over-writing of original routine
**/
void Compass::calibrate()
{
	delay(15000);
  	Wire.beginTransmission(com_slave);
  	Wire.write(com_calibration);
  	Wire.endTransmission();
  	delay(7);
  	delay(40000);
  	Wire.beginTransmission(com_slave);
  	Wire.write(com_exit);
  	Wire.endTransmission();
  	delay(15);
}

/**
*	Compass calibration broken into two steps - start and end
*	for use with other functions (movement) to create
*	auto-calibration routines for robot.
*	Use in conjunction with calibrate_end() for complete function
**/
void Compass::calibrate_start()
{
	Wire.beginTransmission(com_slave);
  	Wire.write(com_calibration);
  	Wire.endTransmission();
  	delayMicroseconds(20);
 }
 
/**	
*	Use with calibrate_start() for full calibration routine
*	If calibrate_end() is not called results may be unpredictible
**/
 void Compass::calibrate_end()
 {
 	Wire.beginTransmission(com_slave);
  	Wire.write(com_exit);
  	Wire.endTransmission();
  	delay(15);
}

/**
*	Polls compass and returns internally corrected compass heading in degrees
*	Internal compass default averages 4 readings.
*	Function receives optional user input to offset headings
**/	
float Compass::heading()
{
	//Variables
	byte MSB,LSB; //Most signifcant bit and least significant bit
	float heading; //Computed heading value
	
	//Send request
	Wire.beginTransmission(com_slave);
	Wire.write(com_read);
	Wire.endTransmission();
	
	//Wait 100 µs for compass to take reading
	delayMicroseconds(700);
	
	//Receive data
	Wire.requestFrom(com_slave,(byte)2); //Request 2 bytes (MSB and LSB)
	MSB = Wire.read();
	LSB = Wire.read();
	
	//Compute heading
	heading = (MSB << 8) + LSB; //Combine MSB and LSB to form compass heading in 10ths of a degree
	
	//Return value
	return util->pmod(heading/10.0,360);
}

///////////////////
////LEGACY CODE////
///////////////////

// const int EEPromLoc = 0x72;
// const int EEPromWriteLoc = 0x77;
// const int RAMLoc = 0x67;
// const int SlaveAddress = 0x00;
// const int XMagOffsetMSB = 0x01;
// const int XMagOffsetLSB = 0x02;
// const int YMagOffsetMSB = 0x03;
// const int YMagOffsetLSB = 0x04;
// const int TimeDelay = 0x05;
// const int SumNum = 0x06;
// const int SoftwVer = 0x07;
// const int OpMode = 0x08;
// const int RAMOpModeLoc = 0x74;
// const int RAMoutDatMod = 0x4E;


/**
*	Uses heading() function and averages over 's' iterations
*	Results are unstable across 0-360 degree transition
**/
// float Compass::avgHeading(int s){
// 	float headCount = 0;
// 	for(int i=0; i<s; i++){
// 		headCount = headCount+heading();
// 	}
// 	return headCount/s;
// }

// /**
// *	Returns binary bytes (must print as BIN in Arduino environment)
// *	of EEProm on compass chip.  All locations can be altered using companion
// *	writeEEProm() function.  Results of manually correcting magnetic
// *	offsets were indeterminate and compass performance was unstable
// *	so use with caution.  See HW6532 Documentation for more information.
// **/
// byte Compass::readEEProm(int s){
// 	byte epromRV;
// 	Wire.beginTransmission(com_slave);
// 	Wire.write(EEPromLoc);
// 		 switch (s){
// 			case 0 : Wire.write(SlaveAddress);
// 			break;
// 			case 1 : Wire.write(XMagOffsetMSB);
// 			break;
// 			case 2 : Wire.write(XMagOffsetLSB);
// 			break;
// 			case 3 : Wire.write(YMagOffsetMSB);
// 			break;
// 			case 4 : Wire.write(YMagOffsetLSB);
// 			break;
// 			case 5 : Wire.write(TimeDelay);
// 			break;
// 			case 6 : Wire.write(SumNum);
// 			break;
// 			case 7 : Wire.write(SoftwVer);
// 			break;
// 			case 8 : Wire.write(OpMode);
// 			break;
// 			}
// 	Wire.endTransmission();
// 	delay(7);
// 	Wire.requestFrom(com_slave, 1);
// 	epromRV = Wire.read();
// 	return epromRV;
// }
// 
// /**
// *	Writes to actual EEProm locations on chip.  Use with caution
// *	as EEProm is obviously non-volatile and improper us can cause
// *	compass instability
// **/
// void Compass::writeEEProm(int s, byte promDat){
// 	Wire.beginTransmission(com_slave);
// 	Wire.write(EEPromWriteLoc);
// 	switch (s){
// 			case 0 : Wire.write(SlaveAddress);
// 			break;
// 			case 1 : Wire.write(XMagOffsetMSB);
// 			break;
// 			case 2 : Wire.write(XMagOffsetLSB);
// 			break;
// 			case 3 : Wire.write(YMagOffsetMSB);
// 			break;
// 			case 4 : Wire.write(YMagOffsetLSB);
// 			break;
// 			case 5 : Wire.write(TimeDelay);
// 			break;
// 			case 6 : Wire.write(SumNum);
// 			break;
// 			case 7 : Wire.write(SoftwVer);
// 			break;
// 			case 8 : Wire.write(OpMode);
// 			break;
// 			}
// 	Wire.write(promDat);
// 	Wire.endTransmission();
// 	delay(7);
// }
// 
// /**
// 	readRAM() reads the Operation Mode byte stored
// 	in RAM (volatile).  This byte is mirrored in EEProm
// 	but changes to RAM are not written to EEProm
// **/
// byte Compass::readRAM(){
// 	byte ramRV;
// 	Wire.beginTransmission(com_slave);
// 	Wire.write(RAMLoc);
// 	Wire.write(RAMOpModeLoc);
// 	delay(7);
// 	Wire.requestFrom(com_slave, 1);
// 	ramRV = Wire.read();
// 	return ramRV;
// }




