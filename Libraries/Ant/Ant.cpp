/** 
*	Library for all higher-level (requiring multiple base libraries) AntBot functions
*	on Arduino platform
*	Joshua Hecker
**/
 
#include <Ant.h>

/**
*	Empty constructor
**/
Ant::Ant(){}

/**
*	Constructor receives all relevant instantiated base objects
**/
Ant::Ant(Compass &co, Movement &m, SoftwareSerial &sS, Ultrasound &ul, Utilities &ut, 
		Location &aL, Location &gL, Location &tL,
		unsigned long &gT,const float &nR,const float &rR,const float &cD,const float &mR,byte &tS,bool &mC) {
		
	//Local objects
	compass = &co;
	move = &m;
	softwareSerial = &sS;
	us = &ul;
	util = &ut;
	
	//Local structs
	absLoc = &aL;
	goalLoc = &gL;
	tempLoc = &tL;
	
	//Local variables
	globalTimer = &gT;
	nestRadius = &nR;
	robotRadius = &rR;
	collisionDistance = &cD;
	usMaxRange = &mR;
	tagStatus = &tS;
	motionCapture = &mC;
}

/**
*	Aligns from current heading to new heading at motor speed
*	We perform multiple iterations to deal with drift using count (default value is 1)
**/
void Ant::align(float newHeading, byte speed, int count) {
	//If using motion capture to control robot
	if (*motionCapture) {
		//Ask iDevice to enable motion capture orientation updates
		String msg = "align" + String((int)newHeading);
		softwareSerial->println(msg);
		serialFind("align");
		
		//Receive updates
		getDirections(60,5000);
	}
	
	//Otherwise
	else {
		for(int i=0; i<count; i++) {
			float currentHeading = compass->heading();
			while (fabs(util->angle(currentHeading,newHeading)) >= 1) {
				if (util->angle(currentHeading,newHeading) <= -1) {
					move->rotateLeft(speed);
					currentHeading = compass->heading();
				}
				if (util->angle(currentHeading,newHeading) >= 1) {
					move->rotateRight(speed);
					currentHeading = compass->heading();
				}
			}
			move->stopMove();
		}
	}
}

/**
*	Continually perturbs motor speed, while rotating in a circle, with the goal
*	of achieving a particular angular velocity in degrees per millisecond
*	(received as user input); returns the motor speed found after several rotations
**/

byte Ant::calibrateSpeed(int degrees) {
	//Variables
	byte speed = 40;
	float angularVelocity = 0;
	float oldHeading, newHeading;
	unsigned long oldTime, newTime;
	oldHeading = compass->heading();
	oldTime = micros();
	
	while (fabs(angularVelocity - degrees/1000.0) > .001) {
		//Accelerate or deccelerate depending on difference from requested velocity
		if ((angularVelocity - degrees/1000.0) < 0) {
			move->rotateLeft(speed++);
		}
		else if ((angularVelocity - degrees/1000.0) > 0) {
			move->rotateLeft(speed--);
		}
		else {
			break;
		}
		
		//Update angular velocity
		newHeading = compass->heading();
		newTime = micros();
		angularVelocity = fabs(util->angle(oldHeading,newHeading))/(newTime-oldTime)*1000;
		oldHeading = newHeading;
		oldTime = newTime;
		
		//100 ms delay to accommodate sensor error
		delay(100);
	}
	
	move->stopMove();
	return speed;
}

/**
*	Auto-calibration of compass rotating at given motor speed
**/
void Ant::calibrateCompass() {
	//Container for message to be passed to iDevice
	String msg;
	
	//Wait 10 seconds to allow setup
	unsigned long timer = millis();
	while (millis()-timer < 10000) {
		msg = "displayCompass calibration: start in ";
		msg += String((int)ceil((10000 - (millis()-timer))/1000.0));
		msg += " secs";
		softwareSerial->println(msg);
	}
	
	//enable calibration mode on compass
	compass->calibrate_start();
	
	//Wait 20 seconds for 2 full rotations of robot
	timer = millis();
	while (millis()-timer < 20000) {
		msg = "displayCompass calibrating: ";
		msg += String((int)ceil((20000 - (millis()-timer))/1000.0));
		msg += " secs remaining";
		softwareSerial->println(msg);
	}
	
	//exit calibration mode on compass
	compass->calibrate_end();
	//instruct iDevice to display calibration stop message
	msg = "displayCompass calibration: done";
	softwareSerial->println(msg);
}

/**
*	Basic function to avoid obstacles/walls
*	Arguments are: speed to return to and pointer to current timer
**/
void Ant::collisionAvoidance(float speed, unsigned long &loopTimer) {
	//Update absolute location with ground covered since last absolute location update
	*absLoc = *absLoc + Location(Utilities::Polar((millis()-loopTimer)*37.7/1000, tempLoc->pol.theta));
	
	//Loop as long as object is found within collisionDistance
	while (us->collisionDetection(*collisionDistance)) {	
		//Check for objects within collisionDistance while also compensating for change in facing angle
		while (us->collisionDetection(*collisionDistance * 2)) {
			move->rotateRight(60);
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
		move->forward(speed,speed);
		delay(*collisionDistance/37.7*1000);
		move->stopMove();
		
		//Update absolute location with ground covered during avoidance behavior above
		*absLoc = *absLoc + Location(Utilities::Polar(*collisionDistance,compass->heading()));
		
		//Update relative location with distance and angle between absolute location and goal
		*tempLoc = *goalLoc - *absLoc;
		
		//Align toward goal
		align(tempLoc->pol.theta,60);
		
		//Reset timer
		util->tic(tempLoc->pol.r/37.7*1000);
	}
	
	//Reset loopTimer
	loopTimer = millis();
}

/**
*	Counts total number of tags within a 360 sweep of current location
*	Receives number of original tag to avoid double counting
**/
int Ant::countNeighbors(int firstTag)
{
	//Initialize local variables
	int tagCount = 0;
	int lastTag = 0;

	for (int i=1; i<12; i++) {
		align(util->pmod(compass->heading() + 30,360),60);
		if (serialFind("new","old",200) == 1) {
			int currentTag = softwareSerial->parseInt();
			softwareSerial->read();
			if ((currentTag != firstTag) && (currentTag != lastTag)) {
				//Count neighboring tag
				tagCount++;
				//Update last tag
				lastTag = currentTag;
			}
		}
	}
	
	return tagCount;
}

/**
*	Maintains headingwhile moving forward at speed by slowing down inner tread 
*	and speeding up outside tread in proportion to angular error from desired heading
**/
void Ant::driftCorrection(byte speed) {
	//Angle offset from correct heading
	float angle = util->angle(compass->heading(),tempLoc->pol.theta);
	
	if (angle >= 5) {
		move->forward(speed,constrain(speed-(angle*30),0,255));
	}
	else if (angle <= -5) {
		move->forward(constrain(speed+(angle*30),0,255),speed);
	}

	//Return to normal speed
	move->forward(speed,speed);
}

/**
*	Moves the robot from its current location using tempLoc's pol.r (distance) and pol.theta (heading)
*	**Assumes fixed velocity of 37.7 cm/s**
**/
void Ant::drive(byte speed, Utilities::EvolvedParameters &ep, Random &r, bool goingHome) {
	randm = &r;
	
    //loopTimer is used to measure ground distance covered during each iteration of while loop
    //This timer is reset after each execution of collisionAvoidance
    unsigned long loopTimer = millis();
    
    //probabilityTimer is used to ensure probabilistic stop checks occur only every 1/4 of a second
    //This timer is reset after 250 milliseconds (or more) pass
    unsigned long probabilityTimer = millis();
    
    //Align to heading
    align(tempLoc->pol.theta,60);
    
    //Set timer to distance assuming velocity of 37.7 cm/s
    util->tic(tempLoc->pol.r/37.7*1000);
    
    //Drive while adjusting for detected objects and motor drift
    while (1) {
    	driftCorrection(120); //correct for motor drift
    	collisionAvoidance(120,loopTimer); //check for collision; maneuver and update location

    	//If the timer has expired (i.e. we have reached our goal location)
    	if (util->isTime()) {
    		//Then exit loop
    		break;
    	}
    	
		//If we're not driving back to the nest, and at least 250 ms have passed
		if (!goingHome && ((millis() - probabilityTimer) >= 250))
		{
			//If tag status is 0, then we are uninformed and stop driving probabilistically
			if ((*tagStatus == 0) && (randm->uniform() < ep.walkDropRate)) {
				break;
			}
			//If tagStatus is 2, then we are following a pheromone trail and we have a small chance of
			//	leaving the trail before we reach the end
			if ((*tagStatus == 2) && (randm->uniform() < ep.trailDropRate)) {
				break;
			}
			
			probabilityTimer = millis();
		}
    }
    
    //Stop movement
    move->stopMove();
}

/**
*	Receives two-byte tuples from the iDevice and translates them into movement
*	Function terminates within a set threshold (returns true) or if timeout occcurs (returns false)
**/
bool Ant::getDirections(byte speed, long timeout) {
	//Local variables
	unsigned long time;
	int cmd[2] = {INT_MAX, INT_MAX};
	bool stopFlag = false;
	
	while (1) {
		util->tic(timeout);
		time = millis();
		
		while ((softwareSerial->read() == -1) && (!util->isTime())) {
			if (((abs(cmd[0]) < INT_MAX) && (abs(cmd[0]) > 0) && (millis() - time > max(abs(cmd[0]),30))) ||
				((cmd[0] == 0) && (millis() - time > max(abs(cmd[1]/2),30)))) {
				move->stopMove();
				cmd[0] = INT_MAX;
			}
		}
		
		if (util->isTime()) {
			move->stopMove();
			return false;
		}

		cmd[0] = softwareSerial->parseInt();
		
		while ((softwareSerial->read() == -1) && (!util->isTime())) {}
		
		if (util->isTime()) {
			move->stopMove();
			return false;
		}
		 
		cmd[1] = softwareSerial->parseInt();
		softwareSerial->read();
		
		if (abs(cmd[0]) < 2) {
			cmd[0] = 0;
			
			if (abs(cmd[1]) < 2) {
				if (stopFlag) {
					move->stopMove();
					return true;
				}
				else {
					stopFlag = true;
				}
			}
			else {
				if (cmd[1] < 0) {
					move->backward(speed,speed);
				}
				else if (cmd[1] > 0) {
					move->forward(speed,speed);
				}
				else {
					move->stopMove();
				}
				stopFlag = false;
			}
		}
		
		else if (abs(cmd[0]) < INT_MAX) {		
			if (cmd[0] < 0) {
				move->rotateRight(speed);
			}
			else if (cmd[0] > 0) {
				move->rotateLeft(speed);
			}
			else {
				move->stopMove();
			}
			stopFlag = false;
		}
		
		else {
			move->rotateLeft(speed);
			stopFlag = false;
		}
	}
}

/**
*	Search for nest using OpenCV through iDevice
*	Reset absolute location via compass and ultrasound measurements
**/
void Ant::localize(byte speed) {

	softwareSerial->println("nest on");
	serialFind("nest on");
	
	getDirections(speed);
	
	softwareSerial->println("nest off");
	serialFind("nest off");
	
	//Read nest distance from iDevice (estimated using OpenCV)
	int cvDistance = softwareSerial->parseInt();

	//We poll the compass here and throw the value away
	//This ensures the correct value for the location calculation below
	compass->heading();
	//****NOTE****
	//Figure out why the compass gives the wrong value here. 
	//Does this happen anywhere else?? (see collision_avoidance() above)
	
	//Read nest distance from ultrasound 
	int usDistance = us->distance();
	
	//If distance is beyond maximum range of ultrasound
	if (usDistance >= *usMaxRange) {
		//we use the distance estimated by our computer vision
		*tempLoc = Location(Utilities::Polar(cvDistance,compass->heading()));
	}
	
	//Otherwise
	else {
		//we use the ultrasound distance
		*tempLoc = Location(Utilities::Polar(usDistance,compass->heading()));
	}
  
	//Shift object localization to subject localization
	*tempLoc = Location(Utilities::Polar(tempLoc->pol.r,util->pmod(tempLoc->pol.theta-180,360)));
	
	//Adjust to account for radius of object
	*tempLoc = *tempLoc + Ant::Location(Utilities::Polar(*nestRadius,tempLoc->pol.theta));
	
	//Adjust to account for distance from ultrasound to robot's center of rotation
	*tempLoc = *tempLoc + Ant::Location(Utilities::Polar(*robotRadius,tempLoc->pol.theta));
	
	*absLoc = *tempLoc;
}

/**
*	Dump comma-separated sensor data to coordination server
*	Current format: time,x,y
*	An optional additional string is received as input and appended to main message
*	Note that MAC address is appended by iDevice
**/
void Ant::print(String info) {
	String msg = "print"
					+ String(micros()-*globalTimer) 
					+ ","
					+ String((int)round(absLoc->cart.x)) 
					+ ","
					+ String((int)round(absLoc->cart.y));
	
	//If optional additional string is provided
	if (info.length() > 0) {
		//append it to the main string
		msg += "," + info;
	}

	softwareSerial->println(msg);
}

/**
*	Performs biased random walk across area
*	Returns boolean representing whether food has been found or not
*	**Assumes fixed velocity of 25 cm/s**
**/
int Ant::randomWalk(Utilities::EvolvedParameters &ep,Random &r,byte speed, float fenceRadius) {
	//Initialization
	int count = 1;
	float goalHeading;
	float currentHeading;
	int tagNum;
	int tagNeighbors;
	int stepTimer = 300; //length of step in random walk (ms)
	int localizationCounter = 0; //time of last localization
	randm = &r;
	
	//Take steps in a random walk, stop walking with probability ep.searchGiveupRate
	//New angle is selected from normal distribution with mean = previous angle
	while (randm->uniform() >= ep.searchGiveupRate) {
	
		currentHeading = compass->heading();
		
		//If food was previously found at this location (either via site fidelity or pheromones)
		if (*tagStatus > 0) {
			//calculate additional deviation
			float deviation = (ep.dirDevCoeff / pow(count,ep.dirTimePow));
			//start with wide turning radius and shrink over time
			goalHeading = util->pmod(randm->normal(currentHeading,util->rad2deg(ep.dirDevConst + deviation)),360);
		}
		//Otherwise
		else {
			//use constant deviation
			goalHeading = util->pmod(randm->normal(currentHeading,util->rad2deg(ep.dirDevConst)),360);
		}
		
		//Likelihood of localizing increases exponentially until it occurs
		if (randm->uniform() < util->expCDF(localizationCounter,0.02)) {
			//Localize to adjust for error
    		localize(60);
			
			//Reset timer
			localizationCounter = 0;
		}
		
		//We add bias to our new heading to direct the robot back towards the nest, but only
		// if absLoc->pol.r is greater than fenceRadius (i.e. we are outside the virtual fence)
		float angle = util->angle(goalHeading, util->pmod(absLoc->pol.theta-180,360));
		goalHeading = util->pmod(goalHeading + util->expCDF((absLoc->pol.r-fenceRadius)/100.0,3)*angle, 360);
		
		//Ask iDevice to enable QR tag searching
		softwareSerial->println("tag on");
		serialFind("tag on");
		
		//Search for tag during alignment
		int result = 0;
		while (abs(util->angle(currentHeading,goalHeading)) >= 3) {
			if ((result = serialFind("new","old",1000)) > 0) {
				break;
			}
			
			currentHeading = compass->heading();
			if (util->angle(currentHeading,goalHeading) >= 30) {
				align(util->pmod(currentHeading + 30,360),60);
			}	
			else if (util->angle(currentHeading,goalHeading) <= -30) {
				align(util->pmod(currentHeading - 30,360),60);
			}
			else {
				align(goalHeading,60);
			}
		}
		
		if (result > 0 || ((result = serialFind("new","old",1000)) > 0)) {
			//If iDevice sent "new"
			if (result == 1) {
				//record tag number
				tagNum = softwareSerial->parseInt();
				softwareSerial->read();
				
				//count number of neighboring tags
				tagNeighbors = countNeighbors(tagNum);
				
				//Ask iDevice to disable QR tag searching
				softwareSerial->println("tag off");
				
				//ensure correct location
				localize(60);
				
				//transmit location and tag info to ABS via iDevice
				print("tag," + String(tagNum) + "," + String(tagNeighbors));
				
				return tagNeighbors;
			}
			
			//If iDevice sent "old"
			else if (result == 2) {
				//Rotate 180 degrees to avoid re-reading tag
				goalHeading = util->pmod(compass->heading()-180,360);
			}
		}
		
		//Ensure iDevice is in tag searching mode
		softwareSerial->println("tag on");
		serialFind("tag on");
		
		//Check collision distance twice to avoid false readings
		if (!us->collisionDetection(*collisionDistance) && !us->collisionDetection(*collisionDistance)) {
			//send dump to server
			print();
			
			//set timer
			util->tic(stepTimer);
			
			//drive forward
			move->forward(speed,speed);
			while (!util->isTime() && !softwareSerial->available()) {}
			move->stopMove();
			
			//if iDevice has discovered a tag and has directions available
			if (softwareSerial->available() && getDirections(60,5000)) {
				//Check buffer for tag status
				result = serialFind("new","old",5000);
				
				//If iDevice sent "new"
				if (result == 1) {
					//record tag number
					tagNum = softwareSerial->parseInt();
					softwareSerial->read();
					
					//count number of neighboring tags
					tagNeighbors = countNeighbors(tagNum);
					
					//Ask iDevice to disable QR tag searching
					softwareSerial->println("tag off");
					
					//ensure correct location
					localize(60);
					
					//transmit location and tag info to server
					print("tag," + String(tagNum) + "," + String(tagNeighbors));
					
					return tagNeighbors;
				}
				
				//If iDevice sent "old"
				else if (result == 2) {
					//Rotate 180 degrees to avoid re-reading tag
					align(util->pmod(compass->heading()-180,360),60);
				}
			}
			
			//update current location with information from last leg
			*absLoc = *absLoc + Location(Utilities::Polar((double)stepTimer/1000*25,goalHeading));

			count++;
			localizationCounter++;
		}
		
		//Ask iDevice to disable QR tag searching
		softwareSerial->println("tag off");
	}
	
	//ensure correct location
	localize(60);

	//negative value indicates no tag found
	return -1;
}

/**
*	Designed as a replacement for the built-in function Serial.find()
*	Checks for a specified message before timing out
**/
int Ant::serialFind(String msg, long timeout) {
	util->tic(timeout);
	String cmd = "";
	
	while (!util->isTime()) {
		char c = softwareSerial->read();
		if (c != -1) {
			cmd += c;
		}

		//return true if msg was found
		if (cmd.endsWith(msg)) {
			return true;
		}
	}
	
	//Timeout must have occured, return false
	return false;
}

/**
*	Overloaded version of serialFind(String msg, int timeout) above
*	Allows for possibility of two different messages
**/
int Ant::serialFind(String msgOne, String msgTwo, long timeout) {
	
	util->tic(timeout);
	String cmd = "";
	
	while (!util->isTime()) {
		char c = softwareSerial->read();
		if (c != -1) {
			cmd += c;
		}

		//return 1 if msgOne was found
		if (cmd.endsWith(msgOne)) {
			return 1;
		}
		
		//return 2 if msgTwo was found
		else if (cmd.endsWith(msgTwo)) {
			return 2;
		}
	}
	
	//Timeout must have occured, return 0
	return 0;
}

// bool Ant::sensorFusion(int n,Location* locs,Covariance* covs,Location &loc_prime,Covariance cov_prime) {
// 	//Instantiate MatrixMath library
// 	MatrixMath math;
// 	
// 	//Create variables:
// 	//accumulators
// 	float loc_sum[1][2] = {0,0};
// 	float cov_sum[covs[0].m][covs[0].n];
// 	//intermediaries
// 	float loc_temp[1][2] = {0,0};
// 	float loc_fused[1][2] = {0,0};
// 	float cov_temp[covs[0].m][covs[0].n];
// 	
// 	//Initialize
// 	for (byte i=0; i<covs[0].m; i++) {
// 		for (byte j=0; j<covs[0].n; j++) {
// 			cov_sum[i][j] = 0;
// 			cov_temp[i][j] = 0;
// 		}
// 	}
// 	
// 	//Perform calculation using Bayesian maximum likelihood estimator
// 	for (byte i=0; i<n; i++) {
// 		//Move required data to temporary storage
// 		loc_temp[0][0] = locs[i].cart.x; loc_temp[0][1] = locs[i].cart.y;
// 		for (byte j=0; j<covs[i].m; j++) {
// 			for (byte k=0; k<covs[i].n; k++) {
// 				cov_temp[j][k] = covs[i].matrix[j][k];
// 			}
// 		}
// 		
// 		//Multiply location by (inverse) covariance matrix, add to accumulator (numerator)
// 		//**if inversion fails (i.e. matrix is singular), return false
// 		if (!math.MatrixInvert((float*)cov_temp,covs[0].m)) {
// 			return false;
// 		}
// 		math.MatrixMult((float*)loc_temp,(float*)cov_temp,1,covs[i].m,covs[i].n,(float*)loc_fused);
// 		math.MatrixAdd((float*)loc_fused,(float*)loc_sum,1,2,(float*)loc_sum);
// 		
// 		//Sum covariance matrices to produce normalizing term (denominator)
// 		math.MatrixAdd((float*)cov_sum,(float*)cov_temp,covs[i].m,covs[i].n,(float*)cov_sum);
// 	}
// 	//Calculate inverse of cov_temp
// 	//**if inversion fails (i.e. matrix is singular), return false
// 	if (!math.MatrixInvert((float*)cov_sum,covs[0].m)) {
// 		return false;
// 	}
// 	//Multiply loc_sum by cov_sum' to produce final location estimate
// 	math.MatrixMult((float*)loc_sum,(float*)cov_sum,1,covs[0].m,covs[0].n,(float*)loc_temp);
// 	
// 	//Assign output variables
// 	loc_prime = Location(Utilities::Cartesian(loc_temp[0][0],loc_temp[0][1]));
// 	for (byte i=0; i<covs[0].m; i++) {
// 		for (byte j=0; j<covs[0].n; j++) {
// 			cov_prime.matrix[i][j] = cov_sum[i][j];
// 		}
// 	}
// 	cov_prime.m = covs[0].m;
// 	cov_prime.n = covs[0].n;
// 	
// 	return true;
// }