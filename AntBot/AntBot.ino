//////////////////////////
////Required Libraries////
//////////////////////////

//Built-in libraries
#include <SoftwareSerial.h>
#include <Wire.h> //Interface for compass

//Libraries acquired from elsewhere
#include <Random.h>

//Libraries created for the AntBot project
#include <Ant.h>
#include <Compass.h>
#include <Movement.h>
#include <Ultrasound.h>
#include <Utilities.h>

////////////////
////Settings////
////////////////

//Simulator (set flag to true for simulator mode, false otherwise)
bool simFlag = false;

//Motion Capture (set flag to true if use motion capture to control robot, false otherwise)
bool mocapFlag = true;

//Parameters evolved by GA
float walkDropRate = 0.133447;
float searchGiveupRate = 0.008831;
float trailDropRate = 0.001625;		
float dirDevConst = 0.538335;
float dirDevCoeff = 2.189899;
float dirTimePow = 0.016253;
float densityPatchThreshold = 1.9153;
float densityPatchConstant = 0.893526;
float densityInfluenceThreshold = 7.27226;
float densityInfluenceConstant = -0.111618;
Utilities::EvolvedParameters ep = Utilities::EvolvedParameters(walkDropRate, searchGiveupRate, trailDropRate, dirDevConst, dirDevCoeff, dirTimePow, 
                                  densityPatchThreshold, densityPatchConstant, densityInfluenceThreshold, densityInfluenceConstant);

//Food
byte tagStatus = 0; //indicates whether tag has been found while searching (0 = no tag, 1 = tag found, 2 = pheromone received)
int tagNeighbors = -1; //holds number of neighboring tags found near tag (negative value indicates no tag found)

//Location
Ant::Location absLoc; //holds absolute location (relative to nest)
Ant::Location goalLoc; //holds current goal location
Ant::Location tempLoc; //holds current location (relative to current leg)
Ant::Location foodLoc; //holds location of last food found
const float fenceRadius = 125; //radius of virtual fence (cm)

//Servos
const byte speed_right = 3; //Ardumoto speed, right side
const byte speed_left = 11; //Ardumoto speed, left side
const byte dir_right = 12; //Ardumoto direction, right side
const byte dir_left = 13; //Ardumoto direction, left side

//SoftwareSerial
const byte ssRx = 4;
const byte ssTx = 5;

//Ultrasound
const byte usEcho = 16; //Ultrasonic echo pin
const byte usTrigger = 17; //Ultrasonic trigger pin
const float usMaxRange = 300; //limit of ultrasound (cm) (we ignore any values returned above this threshold)
const float collisionDistance = 30; //threshold distance for collision recognition (cm)
const float nestRadius = 8; //radius of the nest (cm)

//Other
unsigned long globalTimer; //holds start time of current run

////////////////////////////
////Class Instantiations////
////////////////////////////

SoftwareSerial softwareSerial(ssRx,ssTx);
Utilities util;
Compass compass = Compass(util,softwareSerial,mocapFlag);
Movement move = Movement(speed_right,speed_left,dir_right,dir_left,simFlag);
Ultrasound us = Ultrasound(usTrigger,usEcho,simFlag,usMaxRange);
Random randm;
Ant ant = Ant(compass,move,softwareSerial,us,util,absLoc,goalLoc,tempLoc,globalTimer,nestRadius,collisionDistance,usMaxRange,tagStatus,mocapFlag);


/////////////
////Setup////
/////////////

void setup()
{
  //Open serial connection to iDevice
  softwareSerial.begin(9600);
  
  //Request random seed
  softwareSerial.println("seed");
  ant.serialFind("seed");
  
  //Start prng with received value
  randomSeed(softwareSerial.parseInt());

  //Start global timer
  globalTimer = micros();
  
  //Localize to find starting point
  ant.localize(50);
}

/////////////////
////Main Loop////
/////////////////

void loop()
{
  //We use four location structs:
  //1. goalLoc will hold the location of the goal in relation to the nest (NOTE: A new goal is randomly selected here if food was not found in last search)
  if (tagStatus == 0) goalLoc = Ant::Location(Utilities::Polar(fenceRadius,randm.boundedUniform(0,359)));
  else goalLoc = foodLoc;
  //2. tempLoc holds distance and heading from the *start* of the current leg to the goal
  tempLoc = goalLoc - absLoc;
  //3. absLoc holds the location of the robot relative to the nest
  //4. foodLoc holds the location of any food discovered while searching
  
  //Dump to ABS
  ant.print();
    
  //Drive to goal
  ant.drive(120,ep,randm,false);
  
  //Perform random walk with varying turn radius depending on whether food was found on previous trip
  tagNeighbors = ant.randomWalk(ep,randm,60,fenceRadius);
  
  //If tagNeighbors is 0 or more
  if ((tagNeighbors >= 0) && (randm.uniform() <= (tagNeighbors/ep.densityPatchThreshold + ep.densityPatchConstant))) {
    //Then a tag was collected
    tagStatus = 1;
    //Record its location
    foodLoc = absLoc;
  }
  //Otherwise
  else {
    //No tags were collected
    tagStatus = 0;
  }
  
  //Adjust location structs
  goalLoc = Ant::Location(Utilities::Polar(nestRadius+collisionDistance,absLoc.pol.theta));
  tempLoc = goalLoc - absLoc;
  
  //Drive to nest
  ant.drive(120,ep,randm,true);
  
  //Signal ABS via iDevice of arrival at nest
  ant.print("home");
 
  if (randm.uniform() >= (tagNeighbors/ep.densityInfluenceThreshold + ep.densityInfluenceConstant))
  {
    //Request virtual pheromone location from iDevice
    softwareSerial.println("pheromone");
    
    //If timeout occurs, we assume no location is available
    if (ant.serialFind("pheromone"))
    {
      foodLoc.cart.x = softwareSerial.parseInt();
      softwareSerial.read();
      foodLoc.cart.y = softwareSerial.parseInt();
      softwareSerial.read();
      tagStatus = 2;
    }
  }
}
