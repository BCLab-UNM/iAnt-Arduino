/**
 *	Library for all higher-level (requiring multiple base libraries) AntBot functions
 *	on Arduino platform
 **/

#include <Ant.h>

/**
 *  Empty constructor
 **/
Ant::Ant(){}

/**
 *	Constructor receives all relevant instantiated base objects
 **/
Ant::Ant(Compass &co, Movement &m, Ultrasound &ul, Utilities &ut,
         const float &cD,bool &mC,byte &tSp,byte &rSp,const float &tVe) {
	
	//Local objects
	compass = &co;
	move = &m;
	us = &ul;
	util = &ut;
	
	//Local variables
	collisionDistance = &cD;
	motionCapture = &mC;
	travelSpeed = &tSp;
	rotateSpeed = &rSp;
	travelVelocity = &tVe;
}

/**
 * New functions
 **/

void Ant::adjustMotors(int rotate, int forward, int duration) {
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

	if(!duration) {
		return;
	}

	delay(duration);
	move->stopMove();
}

void Ant::align(float heading) {
	for(int i = 0; i < 10; i++) {
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
	float direction = getCompass();

	util->tic(distance / speed * 1000);

	move->forward(speed, speed);

	//Drive while adjusting for detected objects and motor drift
	while(1) {
		driftCorrection(direction); //correct for motor drift

		float distanceLeft = (util->timerStop - millis()) / 1000 * speed;
		direction = collisionAvoidance(direction, distanceLeft); //check for collision; maneuver and update location

		//If the timer has expired (i.e. we have reached our goal location)
		if(util->isTime()) break;
	}

	move->stopMove();
}

float Ant::getCompass() {
	return compass->heading();
}

float Ant::getUltrasound() {
	return us->distance();
}

/**
 *  Avoid obstacles.
 *  Input: direction and distance to target.
 *  Output: new direction to target after avoidance.
 *
 *  Calls util->tic to adjust for new distance to target.
 **/
float Ant::collisionAvoidance(float direction, float distance) {

	Location delta, path;
	path = Location(Utilities::Polar(distance, direction));

	//Loop as long as object is found within collisionDistance
	while(us->collisionDetection(*collisionDistance)) {

		//Check for objects within collisionDistance while also compensating for change in facing angle
		while(us->collisionDetection(*collisionDistance * 1.5)) {
			move->rotateRight(*rotateSpeed);
		}
		
		//Wait another half second to ensure horizontal clearance
		delay(500);
		move->stopMove();
		
		//We poll the compass here and throw the value away
		//This ensures the correct value for the location calculation below
		compass->heading();
		
		//Move to empty location in space discovered above
		move->forward(*travelSpeed, *travelSpeed);
		delay((*collisionDistance / *travelVelocity) * 1000);
		move->stopMove();
		
		//Vector defining ground covered during avoidance behavior above
		delta = Location(Utilities::Polar(*collisionDistance, compass->heading()));
		
		//Update relative location with distance and angle between previous path and new, changed path during avoidance
		path = path - delta;
		
		//Align toward goal
		align(path.pol.theta);
		
		//Reset timer
		util->tic((path.pol.r / *travelVelocity) * 1000);
	}

	return path.pol.theta;
}

/**
 *	Maintains heading while moving forward by slowing down inner tread and 
 *  speeding up outside tread in proportion to angular error from desired heading
 **/
void Ant::driftCorrection(float direction) {
	int allowedOffset;
	
	//Angle offset from correct heading
	float angle = util->angle(getCompass(), direction);
	
	//If using motion capture to control robot
	if(*motionCapture) {
		//Use a looser offset to compensate for delay
		allowedOffset = 5;
	}
	//Otherwise
	else {
		//Use tight offset
		allowedOffset = 1;
	}
	
	if(angle >= allowedOffset) {
		move->forward(*travelSpeed,constrain(*travelSpeed - (angle * 25), 0, 255));
	}
	else if(angle <= -allowedOffset) {
		move->forward(constrain(*travelSpeed + (angle * 25), 0, 255), *travelSpeed);
	}
	else {
		move->forward(*travelSpeed,* travelSpeed);
	}
}