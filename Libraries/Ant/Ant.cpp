/**
 *	Library for all higher-level (requiring multiple base libraries) AntBot functions
 *	on Arduino platform
 **/

#include <Ant.h>

/**
 *	Empty constructor
 **/
Ant::Ant(){}

/**
 *	Constructor receives all relevant instantiated base objects
 **/
Ant::Ant(Compass &co, Movement &m, Random &r, SoftwareSerial &sS, Ultrasound &ul, Utilities &ut,
         Location &aL, Location &gL, Location &tL, Utilities::EvolvedParameters &ep,
         unsigned long &gT,const float &nR,const float &rR,const float &cD,
         const float &mR,byte &iS,bool &mC,byte &tSp,byte &rSp,const float &tVe) {
    
	//Local objects
	compass = &co;
	move = &m;
	randm = &r;
	softwareSerial = &sS;
	us = &ul;
	util = &ut;
	
	//Local structs
	absLoc = &aL;
	goalLoc = &gL;
	tempLoc = &tL;
	evolvedParams = &ep;
	
	//Local variables
	globalTimer = &gT;
	nestRadius = &nR;
	robotRadius = &rR;
	collisionDistance = &cD;
	usMaxRange = &mR;
	informedStatus = &iS;
	motionCapture = &mC;
	travelSpeed = &tSp;
	rotateSpeed = &rSp;
	travelVelocity = &tVe;
}

/**
 * New functions
 **/

void Ant::adjustMotors(int rotate, int forward) {
	if(!rotate && !forward) {
		return move->stopMove();
	}

	if(!rotate) {
		if(forward < 0) {
			move->backward(*travelSpeed, *travelSpeed);
		}
		else {
			move->forward(*travelSpeed, *travelSpeed);
		}
	}
	else {
		if(rotate < 0) {
			move->rotateRight(*rotateSpeed);
		}
		else {
			move->rotateLeft(*rotateSpeed);
		}
	}
}

void Ant::align(float heading, int count) {
	for(int i = 0; i < count; i++) {
		float currentHeading = compass->heading();
		while(fabs(util->angle(currentHeading, heading)) >= 1) {
			if(util->angle(currentHeading, heading) <= -1) {
				move->rotateLeft(*rotateSpeed);
				currentHeading = compass->heading();
			}

			if(util->angle(currentHeading, heading) >= 1) {
				move->rotateRight(*rotateSpeed);
				currentHeading= compass->heading();
			}
		}

		move->stopMove();
	}
}

void Ant::drive(float distance) {

	float speed = *travelVelocity;

	//Set timer to distance
	util->tic(distance / speed * 1000);

	//loopTimer is used to measure ground distance covered during each iteration of while loop
	//This timer is reset after each execution of collisionAvoidance
	unsigned long loopTimer = millis();

	move->forward(*travelSpeed, *travelSpeed);

	//Drive while adjusting for detected objects and motor drift
	while (1) {
		//driftCorrection(); //correct for motor drift
		//collisionAvoidance(loopTimer); //check for collision; maneuver and update location

		//If the timer has expired (i.e. we have reached our goal location)
		if (util->isTime()) {
			move->stopMove();
			//Then exit loop
			break;
		}
	}
	
	//Stop movement
	move->stopMove();
}

float Ant::getCompass() {
	return compass->heading();
}

float Ant::getUltrasound() {
	return us->distance();
}


/**
 *	Basic function to avoid obstacles/walls
 *	Arguments are: speed to return to and pointer to current timer
 **/
void Ant::collisionAvoidance(unsigned long &loopTimer) {
	//Update absolute location with ground covered since last absolute location update
	*absLoc = *absLoc + Location(Utilities::Polar(*travelVelocity*((millis()-loopTimer)/1000.0), tempLoc->pol.theta));
	//Reset loopTimer
	loopTimer = millis();
	
	//Loop as long as object is found within collisionDistance
	while (us->collisionDetection(*collisionDistance)) {
		//Check for objects within collisionDistance while also compensating for change in facing angle
		while (us->collisionDetection(*collisionDistance * 2)) {
			move->rotateRight(*rotateSpeed);
		}
        
		//Wait another half second to ensure horizontal clearance
		delay(500);
		move->stopMove();
		
		//We poll the compass here and throw the value away
		//This ensures the correct value for the location calculation below
		compass->heading();
		//****NOTE****
		//Figure out why the compass gives the wrong value here.
		//Does this happen anywhere else??
		
		//Move to empty location in space discovered above
		move->forward(*travelSpeed,*travelSpeed);
		delay((*collisionDistance/(*travelVelocity))*1000);
		move->stopMove();
		
		//Update absolute location with ground covered during avoidance behavior above
		*absLoc = *absLoc + Location(Utilities::Polar(*collisionDistance,compass->heading()));
		
		//Update relative location with distance and angle between absolute location and goal
		*tempLoc = *goalLoc - *absLoc;
		
		//Align toward goal
		align(tempLoc->pol.theta,5);
		
		//Reset timer
		util->tic((tempLoc->pol.r/(*travelVelocity))*1000);
        
		//Reset loopTimer
		loopTimer = millis();
	}
}

/**
 *	Maintains heading while moving forward by slowing down inner tread and 
 *  speeding up outside tread in proportion to angular error from desired heading
 **/
void Ant::driftCorrection() {
	int allowedOffset;
	
	//Angle offset from correct heading
	float angle = util->angle(compass->heading(),tempLoc->pol.theta);
    
	//If using motion capture to control robot
	if (*motionCapture) {
		//Use a looser offset to compensate for delay
		allowedOffset = 5;
	}
	//Otherwise
	else {
		//Use tight offset
		allowedOffset = 1;
	}
    
	if (angle >= allowedOffset) {
		move->forward(*travelSpeed,constrain(*travelSpeed-(angle*25),0,255));
	}
	else if (angle <= -allowedOffset) {
		move->forward(constrain(*travelSpeed+(angle*25),0,255),*travelSpeed);
	}
	else {
		move->forward(*travelSpeed,*travelSpeed);
	}
}
