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
		unsigned long &gT, const float &nR)
{
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
}

/**
*	Aligns from current heading to new heading at motor speed s
*	We perform multiple iterations to deal with drift using count (default value is 1)
**/
void Ant::align(float newHeading, byte speed, int count)
{
	float currentHeading = compass->heading();

	for(int i=0; i<count; i++)
	{
		while (fabs(util->angle(currentHeading,newHeading)) >= 1)
		{
			while (util->angle(currentHeading,newHeading) <= -1)
			{
			  move->rotateLeft(speed);
			  currentHeading = compass->heading();
			}
			while (util->angle(currentHeading,newHeading) >= 1)
			{
			  move->rotateRight(speed);
			  currentHeading = compass->heading();
			}
			move->stopMove();
			currentHeading = compass->heading();
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
void Ant::calibrateCompass(byte speed) {
	compass->calibrate_start();
	move->rotateRight(speed);
	delay(20000);
	compass->calibrate_end();
}

/**
*	Basic function to avoid obstacles/walls
*	Arguments are: distance to object (in cm), speed to return to, and pointer to current timer
**/
void Ant::collisionAvoidance(float boundry,float speed,unsigned long &loopTimer) {
	//Update absolute location with ground covered since last absolute location update
	*absLoc = *absLoc + Location(Utilities::Polar((millis()-loopTimer)*37.7/1000, tempLoc->pol.theta));
	
	//Loop as long as object is found within boundry
	while (us->collisionDetection(boundry)) {	
		delay(50);
		//Check for objects within boundry while also compensating for change in facing angle
		while (us->collisionDetection(boundry*2)) {
			move->rotateRight(80);
			delay(50);
		}
	
		//Wait another half second to ensure horizontal clearance
		delay(500);
		
		//We poll the compass here and throw the value away
		//This ensures the correct value for the location calculation below
		compass->heading();
		//****NOTE****
		//Figure out why the compass gives the wrong value here. 
		//Does this happen anywhere else??
		
		//Move to empty location in space discovered above
		move->forward(speed,speed);
		delay(boundry/37.7*1000);
		move->stopMove();
		
		//Update absolute location with ground covered during avoidance behavior above
		*absLoc = *absLoc + Location(Utilities::Polar(boundry,compass->heading()));
		
		//Update relative location with distance and angle between absolute location and goal
		*tempLoc = *goalLoc - *absLoc;
		
		//Align toward goal
		align(tempLoc->pol.theta,80,5);
		
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
int Ant::countNeighbors(int tagNum)
{
	//Initialize local variables
	int tagCount = 0;
	float start_heading = compass->heading();
	
	//Rotate 30 degrees to avoid re-reading original tag
	align(util->pmod(start_heading + 30,360),70);
	
	//Ask iDevice to enable QR reader
	softwareSerial->println("read on");
	while (!serialFind("read on")) {}
	
	for (int i=1; i<12; i++) {
		align(util->pmod(start_heading + i*30,360),70);
		if ((serialFind("yes","no",500) == 1) && (softwareSerial->parseInt() != tagNum)) {
			//Count neighboring tag
			tagCount++;
			//Restart QR reader (required by QR SDK running on iDevice)
			softwareSerial->println("read off");
			softwareSerial->println("read on");
			while (!serialFind("read on")) {}
		}
	}
	
	//Ask iDevice to disable QR reader
	softwareSerial->println("read off");
	
	return tagCount;
}

/**
*	Corrects from motor drift using gyroscope values read on the iDevice
* 	We use a PID controller to smooth the path of the robot
**/
void Ant::driftCorrection(PID &pid, double &input, double &output, byte speed) {
	//Recieve current rotation rate from iDevice
	input = softwareSerial->parseInt();
	softwareSerial->read();

	//Execute PID
	pid.Compute();
    
    //Move forward, adjusting the power applied to each side of the robot
    //	using the output from the PID
    if (output < 0) {
    	move->forward(speed+output,speed+output);
    }
    else if (output > 0) {
    	move->forward(speed,speed-output);
    }
    else {
    	move->forward(speed,speed);
    }
}

/**
*	Receives two-byte tuples from the iDevice and translates them into movement
*	Function terminates when stop signal is received (returns true) or if timeout occcurs (returns false)
**/
bool Ant::getDirections(byte speed, int timeout) {
	unsigned long time;
	char cmd[2];
	byte b;
	bool flag = false;
	
	while (1) {
		util->tic(timeout);
		time = millis();
		
		while ((b == 255) && (!util->isTime())) {
			b = softwareSerial->read();
			if ((cmd[1] > 0) && (millis() - time > cmd[1])) {
				move->stopMove();
				cmd[1] = 0;
			}
		}

		cmd[0] = b;
		b = 255;
		
		while ((b == 255) && (!util->isTime())) {
			b = softwareSerial->read();
		}
		cmd[1] = b;
		b = 255;
		
		if (util->isTime()) {
			return false;
		}

		//stop
		if (cmd[0] == 's') {
			move->stopMove();
			if (flag) {
				return true;
			}
			else {
				flag = true;
			}
		}
		else {
			//rotate left
			if (cmd[0] == 'l') {
				move->rotateLeft(speed);
			}
			//rotate right
			if (cmd[0] == 'r') {
				move->rotateRight(speed);
			}
			//drive forward
			if (cmd[0] == 'f') {
				move->forward(speed,speed);
			}
			//drive backward
			if (cmd[0] == 'b') {
				move->backward(speed,speed);
			}
			flag = false;
		}
	}
}

/**
*	Search for red-colored nest using OpenCV through iDevice
*	Reset absolute location via compass and ultrasound measurements
**/
Ant::Location Ant::localize(byte speed) {

	softwareSerial->println("nest on");
	while (!serialFind("nest on")) {}
	
	getDirections(speed);
	
	softwareSerial->println("nest off");

	//We poll the compass here and throw the value away
	//This ensures the correct value for the location calculation below
	compass->heading();
	//****NOTE****
	//Figure out why the compass gives the wrong value here. 
	//Does this happen anywhere else?? (see collision_avoidance() above)
		
	*tempLoc = Location(Utilities::Polar(us->distance(),compass->heading()));
  
	//Shift object localization to subject localization
	*tempLoc = Location(Utilities::Polar(tempLoc->pol.r,util->pmod(tempLoc->pol.theta-180,360)));
	
	//Adjust to account for radius of object
	*tempLoc = *tempLoc + Ant::Location(Utilities::Polar(*nestRadius,tempLoc->pol.theta));
	
	return *tempLoc;
}

/**
*	Dump sensor data to coordination server
*	Current format: time,x,y
*	An optional additional string is received as input and appended to main message
**/
void Ant::print(String info) {
	String msg = "print"
					+ String(micros()-*globalTimer) 
					+ ","
					+ String((int)round(absLoc->cart.x)) 
					+ ","
					+ String((int)round(absLoc->cart.y));
	
	//If optional additional string is provided, append it to the main string
	if (info.length() > 0) {
		msg += "," + info;
	}

	softwareSerial->println(msg);
}

/**
*	Performs biased random walk across area
*	Returns boolean representing whether food has been found or not
*	**Assumes fixed velocity of 25 cm/s**
**/
int Ant::randomWalk(Random &r,byte speed,float std,float collisionDistance,float fenceRadius,bool tagFound) {
	//Initialization
	int count = 1;
	float heading;
	int tagNum;
	int tagNeighbors;
	randm = &r;
	
	//Ask iDevice to enable QR tag searching
	softwareSerial->println("tag on");
	while (!serialFind("tag on")) {}
	
	//Take steps in a random walk, stop walking with probability 1/10000
	//New angle is selected from normal distribution with mean = previous angle
	while (random(0,10000) > 0) {	
		//pick random heading from normal distribution
		//if food was previously found at this location, start with wide turning radius and shrink over time
		if (tagFound) {
			heading = util->pmod(randm->normal(compass->heading(),std + 360/pow(count,2)),360);
		}
		//otherwise, use constant 'std' provided
		else {
			heading = util->pmod(randm->normal(compass->heading(),std),360);
		}
		
		//calculate location after next step in random walk
		*tempLoc = *absLoc + Location(Utilities::Polar(25,heading));
		
		//if stepping outside of virtual fence, search for nest to ensure correct location
		if (tempLoc->pol.r > fenceRadius) {
			//Ask iDevice to disable QR tag searching
			softwareSerial->println("tag off");

			//ensure correct location
			*absLoc = localize(speed);
			
			//Ask iDevice to enable QR tag searching
			softwareSerial->println("tag on");
			while (!serialFind("tag on")) {}
		}

		//ensure we are inside virtual fence (otherwise we stay aligned toward the nest)
		if (absLoc->pol.r < fenceRadius) {
			//search for tag during alignment
			while ((abs(util->angle(compass->heading(),heading))>30) && (!softwareSerial->available())){
				if (util->angle(compass->heading(),heading) > 0) {
					align(util->pmod(compass->heading()+30,360),70,5);
				}	
				else {
					align(util->pmod(compass->heading()-30,360),70,5);
				}
			}
			
			//if iDevice has discovered a tag and has directions available
			if (softwareSerial->available() && getDirections(speed)) {
				//Disable tag search, enable tag reading
				softwareSerial->println("tag off");
				softwareSerial->println("read on");
				while (!serialFind("read on")) {}
				
				//Check iDevice for valid QR tag
				int result = serialFind("yes","no",5000);
				
				//If iDevice sent "yes"
				if (result == 1) {
					//record tag number
					tagNum = softwareSerial->parseInt();
					
					//disable tag reading
					softwareSerial->println("read off");
					
					//count number of neighboring tags
					tagNeighbors = countNeighbors(tagNum);
					
					//ensure correct location
					*absLoc = localize(speed);
					
					//transmit location and tag info to server
					print("tag," + String(tagNum) + "," + String(tagNeighbors));
					
					return tagNeighbors;
				}
				
				//If iDevice sent "no"
				else if (result == 2) {
					//Rotate 180 degrees to avoid re-reading tag
					heading = util->pmod(compass->heading()-180,360);
				}
				
				//Disable tag reading, enable tag search
				softwareSerial->println("read off");
				softwareSerial->println("tag on");
				while (!serialFind("tag on")) {}
			}
			
			//Complete alignment
			align(heading,70,5);
		}
		
		//Check collision distance
		if (!us->collisionDetection(collisionDistance)) {
			//length of step in random walk
			int stepTimer = 300;
			
			//send dump to server
			print();
			
			//set timer
			util->tic(stepTimer);
			
			//drive forward
			move->forward(speed,speed);
			while (!util->isTime() && !softwareSerial->available()) {}
			move->stopMove();
			
			//if iDevice has discovered a tag and has directions available
			if (softwareSerial->available() && getDirections(speed)) {
				//Disable tag search, enable tag reading
				softwareSerial->println("tag off");
				softwareSerial->println("read on");
				while (!serialFind("read on")) {}
				
				//Check iDevice for valid QR tag
				int result = serialFind("yes","no",5000);
				
				//If iDevice sent "yes"
				if (result == 1) {
					//record tag number
					tagNum = softwareSerial->parseInt();
					
					//disable tag reading
					softwareSerial->println("read off");
					
					//count number of neighboring tags
					tagNeighbors = countNeighbors(tagNum);
					
					//ensure correct location
					*absLoc = localize(speed);
					
					//transmit location and tag info to server
					print("tag," + String(tagNum) + "," + String(tagNeighbors));
					
					return tagNeighbors;
				}
				
				//If iDevice sent "no"
				else if (result == 2) {
					//Rotate 180 degrees to avoid re-reading tag
					align(util->pmod(compass->heading()-180,360),70,5);
				}
				
				//Disable tag reading, enable tag search
				softwareSerial->println("read off");
				softwareSerial->println("tag on");
				while (!serialFind("tag on")) {}
			}
			
			//update current location with information from last leg
			*absLoc = *absLoc + Location(Utilities::Polar((double)stepTimer/1000*25,heading));

			count++;
		}
	}
	
	//Ask iDevice to disable QR tag searching
	softwareSerial->println("tag off");
	
	//ensure correct location
	*absLoc = localize(speed);

	//negative value indicates no tag found
	return -1;
}

bool Ant::sensorFusion(int n,Location* locs,Covariance* covs,Location &loc_prime,Covariance cov_prime) {
	//Instantiate MatrixMath library
	MatrixMath math;
	
	//Create variables:
	//accumulators
	float loc_sum[1][2] = {0,0};
	float cov_sum[covs[0].m][covs[0].n];
	//intermediaries
	float loc_temp[1][2] = {0,0};
	float loc_fused[1][2] = {0,0};
	float cov_temp[covs[0].m][covs[0].n];
	
	//Initialize
	for (byte i=0; i<covs[0].m; i++) {
		for (byte j=0; j<covs[0].n; j++) {
			cov_sum[i][j] = 0;
			cov_temp[i][j] = 0;
		}
	}
	
	//Perform calculation using Bayesian maximum likelihood estimator
	for (byte i=0; i<n; i++) {
		//Move required data to temporary storage
		loc_temp[0][0] = locs[i].cart.x; loc_temp[0][1] = locs[i].cart.y;
		for (byte j=0; j<covs[i].m; j++) {
			for (byte k=0; k<covs[i].n; k++) {
				cov_temp[j][k] = covs[i].matrix[j][k];
			}
		}
		
		//Multiply location by (inverse) covariance matrix, add to accumulator (numerator)
		//**if inversion fails (i.e. matrix is singular), return false
		if (!math.MatrixInvert((float*)cov_temp,covs[0].m)) {
			return false;
		}
		math.MatrixMult((float*)loc_temp,(float*)cov_temp,1,covs[i].m,covs[i].n,(float*)loc_fused);
		math.MatrixAdd((float*)loc_fused,(float*)loc_sum,1,2,(float*)loc_sum);
		
		//Sum covariance matrices to produce normalizing term (denominator)
		math.MatrixAdd((float*)cov_sum,(float*)cov_temp,covs[i].m,covs[i].n,(float*)cov_sum);
	}
	//Calculate inverse of cov_temp
	//**if inversion fails (i.e. matrix is singular), return false
	if (!math.MatrixInvert((float*)cov_sum,covs[0].m)) {
		return false;
	}
	//Multiply loc_sum by cov_sum' to produce final location estimate
	math.MatrixMult((float*)loc_sum,(float*)cov_sum,1,covs[0].m,covs[0].n,(float*)loc_temp);
	
	//Assign output variables
	loc_prime = Location(Utilities::Cartesian(loc_temp[0][0],loc_temp[0][1]));
	for (byte i=0; i<covs[0].m; i++) {
		for (byte j=0; j<covs[0].n; j++) {
			cov_prime.matrix[i][j] = cov_sum[i][j];
		}
	}
	cov_prime.m = covs[0].m;
	cov_prime.n = covs[0].n;
	
	return true;
}

/**
*	Designed as a replacement for the built-in function Serial.find()
*	Checks for a specified message before timing out
**/
int Ant::serialFind(String msg, int timeout) {
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
int Ant::serialFind(String msgOne, String msgTwo, int timeout) {
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
		if (cmd.endsWith(msgTwo)) {
			return 2;
		}
	}
	
	//Timeout must have occured, return 0
	return 0;
}