/**
*	Library for Utility functions related to AntBot project
*	Created by Joshua Hecker
*	Moses Lab, Department of Computer Science, University of New Mexico
**/

#ifndef Utilities_h
#define Utilities_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
  
class Utilities
{
	public:
		//Constructor
		Utilities();
		
		//Structs
		struct Cartesian {
			Cartesian(float a, float b):x(a),y(b){}
			float x,y;
		};
		struct Polar {
			Polar(float a, float b):r(a),theta(b){}
			float r,theta;
		};
		struct EvolvedParameters {
			EvolvedParameters(float dR, float tDR, float wDR, float sGR, float dDC, float dDC1, 
				float dDC2, float dTP1, float dTP2, float dT, float dS, float dC, float dPT, 
				float dPC, float dIT, float dIC):
				decayRate(dR), trailDropRate(tDR), walkDropRate(wDR), searchGiveupRate(sGR),
				dirDevConst(dDC), dirDevCoeff1(dDC1), dirDevCoeff2(dDC2), dirTimePow1(dTP1),
				dirTimePow2(dTP2), densityThreshold(dT), densitySensitivity(dS), densityConstant(dC),
				densityPatchThreshold(dPT), densityPatchConstant(dPC), densityInfluenceThreshold(dIT),
				densityInfluenceConstant (dIC){}
			float decayRate;
			float trailDropRate;
			float walkDropRate;
			float searchGiveupRate;		
			float dirDevConst;
			float dirDevCoeff1;
			float dirDevCoeff2;
			float dirTimePow1;
			float dirTimePow2;		
			float densityThreshold;
			float densitySensitivity;
			float densityConstant;
			float densityPatchThreshold;
			float densityPatchConstant;
			float densityInfluenceThreshold;
			float densityInfluenceConstant;
		};
		
		//Functions
		float angle(float start_angle, float end_angle);
		float deg2rad(float degree);
		float rad2deg(float radian);
		Utilities::Cartesian pol2cart(Utilities::Polar pol);
		Utilities::Polar cart2pol(Utilities::Cartesian cart);
		float pmod(float dividend, float divisor);
		float expCDF(float x, float lambda=1.0);
		
		//Legacy Functions
		//byte* parseIP(char* address);
		//int availableMemory();
		//Utilities::Polar gps2pol(float lat1, float lon1, float lat2, float lon2);
		//void onLED(byte pinLED);
		//void offLED(byte pinLED);
		
		//Timer functions
		void tic();
		void tic(long timerLength);
		long toc();
		bool isTime();
		
	private:
		//Timer variables
		unsigned long timerStart;
		unsigned long timerStop;
};

#endif