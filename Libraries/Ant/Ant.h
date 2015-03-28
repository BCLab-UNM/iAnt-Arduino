/**
 *  Library for all higher-level (requiring multiple base libraries) AntBot functions
 *  on Arduino platform
 **/

#ifndef Ant_h
#define Ant_h

#include <Compass.h>
#include <Movement.h>
#include <Ultrasound.h>
#include <Utilities.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Ant {

public:

	//Structs
	struct Location {
		Utilities util; //object for use in overloaded operators (below)
		Location():pol(Utilities::Polar(0,0)),cart(util.pol2cart(Utilities::Polar(0,0))) {}
		Location(Utilities::Polar p):pol(p),cart(util.pol2cart(p)) {}
		Location(Utilities::Cartesian c):pol(util.cart2pol(c)),cart(c) {}
		Utilities::Polar pol; //polar coordinates (r,theta)
		Utilities::Cartesian cart; //cartesian coordinates (x,y)
		
		//Operator overloading functions:
		//Overload "plus" operator
		Location operator+ (const Location new_location) {
			return Location(util.cart2pol
			                (Utilities::Cartesian(cart.x+new_location.cart.x,cart.y+new_location.cart.y)));
		}
		
		//Overload "minus" operator
		Location operator- (const Location new_location) {
			return Location(util.cart2pol
			                (Utilities::Cartesian(cart.x-new_location.cart.x,cart.y-new_location.cart.y)));
		}
	};
	
	//Constructors
	Ant();
	Ant(Compass &co,Movement &m,Ultrasound &ul,Utilities &ut,
	    const float &cD,bool &mC,byte &tSp,byte &rSp,const float &tVe);
	
	//Functions
	void adjustMotors(int rotate, int forward, int duration);
	void align(float heading);
	void drive(float distance);
	float getCompass();
	float getUltrasound();
	float collisionAvoidance(float direction, float distance);
	void driftCorrection(float direction);
	
	//Legacy Functions
	//bool sensorFusion(int n,Location* locs,Covariance* covs,Location &loc_prime,Covariance cov_prime);
	
private:
	
	//Pointers to objects
	Compass *compass;
	Movement *move;
	Ultrasound *us;
	Utilities *util;
	
	//Pointers to variables
	const float *collisionDistance;
	bool *motionCapture;
	byte *travelSpeed;
	byte *rotateSpeed;
	const float *travelVelocity;
};

#endif
