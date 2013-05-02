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

//Motion Capture (set flag to true if using motion capture to control robot, false otherwise)
bool mocapFlag = false;

//Parameters evolved by GA
float travelGiveUpProbability = 0.0;
float searchGiveUpProbability = 0.0;	
float uninformedSearchCorrelation = 0.0;
float informedSearchCorrelationDecayRate = 0.0;
float stepSizeVariation = 0.0;
float siteFidelityRate = 0.0;
float pheromoneFollowingRate = 0.0;
Utilities::EvolvedParameters ep = Utilities::EvolvedParameters(travelGiveUpProbability, searchGiveUpProbability, uninformedSearchCorrelation, 
                                  informedSearchCorrelationDecayRate, stepSizeVariation, siteFidelityRate, pheromoneFollowingRate);

//Food
byte tagStatus = 0; //indicates whether tag has been found while searching (0 = no tag, 1 = tag found, 2 = pheromone received)
int tagNeighbors = 0; //holds number of neighboring tags found near tag

//Location
Ant::Location absLoc; //holds absolute location (relative to nest)
Ant::Location goalLoc; //holds current goal location
Ant::Location tempLoc; //holds current location (relative to current leg)
Ant::Location foodLoc; //holds location of last food found
const float fenceRadius = 500; //radius of virtual fence (cm)

//Servos
const byte speed_right = 3; //Ardumoto speed, right side
const byte speed_left = 11; //Ardumoto speed, left side
const byte dir_right = 12; //Ardumoto direction, right side
const byte dir_left = 13; //Ardumoto direction, left side
byte travelSpeed = 255; //motor speed used while moving forward/backward
byte rotateSpeed = 180; //motor speed used while rotating left/right

//SoftwareSerial
const byte ssRx = 4;
const byte ssTx = 5;

//Ultrasound
const byte usEcho = 16; //Ultrasonic echo pin
const byte usTrigger = 17; //Ultrasonic trigger pin
const float usMaxRange = 300; //limit of ultrasound (cm) (we ignore any values returned above this threshold)
const float collisionDistance = 30; //threshold distance for collision recognition (cm)
const float nestRadius = 8; //radius of the nest (cm)
const float robotRadius = 10.5; //distance from ultrasound sensor to robot's center of rotation

//Other
unsigned long globalTimer; //holds start time of current run
const float travelVelocity = 16.3; //speed of robot during travel behavior

////////////////////////////
////Class Instantiations////
////////////////////////////

SoftwareSerial softwareSerial(ssRx,ssTx);
Utilities util;
Compass compass = Compass(util,softwareSerial,mocapFlag);
Movement move = Movement(speed_right,speed_left,dir_right,dir_left,simFlag);
Ultrasound us = Ultrasound(usTrigger,usEcho,simFlag,usMaxRange);
Random randm;
Ant ant = Ant(compass,move,randm,softwareSerial,us,util,absLoc,goalLoc,tempLoc,ep,globalTimer,nestRadius,robotRadius,collisionDistance,usMaxRange,tagStatus,mocapFlag,travelSpeed,rotateSpeed,travelVelocity);


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
}

/////////////////
////Main Loop////
/////////////////

void loop()
{
  //Localize
  ant.localize();
  
  //Signal location to ABS via iDevice
  ant.print("home");
 
  //If timeout occurs, we assume no location is available
  if (ant.serialFind("pheromone"))
  {
    foodLoc.cart.x = softwareSerial.parseInt();
    softwareSerial.read();
    foodLoc.cart.y = softwareSerial.parseInt();
    softwareSerial.read();
    
    //We decide whether to use the pheromone location we've received:
    //1. If no tags were found on the previous trip, we follow pheromones by default
    //2. If we decided to use site fidelity on the previous loop iteration, we don't use pheromones
    //3. We probabilistically decide to use pheromones (and decide not to use site fidelity)
    if ((tagNeighbors == -1) || 
          (((tagStatus != 1) && randm.uniform() < util.exponentialCDF(9 - tagNeighbors, ep.pheromoneFollowingRate)) && (randm.uniform() > util.exponentialCDF(tagNeighbors + 1, ep.siteFidelityRate)))){
      tagStatus = 2;
    }
  }
  
  //goalLoc will holds location of the goal in relation to the nest (NOTE: A new goal is randomly selected here if food was not found in last search)
  if (tagStatus == 0) goalLoc = Ant::Location(Utilities::Polar(fenceRadius,randm.boundedUniform(0,359)));
  else goalLoc = foodLoc;
  //tempLoc holds distance and heading from the *start* of the current leg to the goal
  tempLoc = goalLoc - absLoc;
  //absLoc holds the location of the robot relative to the nest
  //foodLoc holds the location of any food discovered while searching
  
  //Drive to goal
  ant.drive(false);
  
  //Perform random walk with varying turn radius depending on whether food was found on previous trip
  tagNeighbors = ant.randomWalk(fenceRadius);
  
  //We decide whether to use site fidelity:
  //1. If a tag was found, and...
  //2. If we probabilistically decide to use site fidelity (and decide not to use pheromones)
  if ((tagNeighbors >= 0) && (randm.uniform() < util.exponentialCDF(tagNeighbors + 1, ep.siteFidelityRate)) && (randm.uniform() > util.exponentialCDF(9 - tagNeighbors, ep.pheromoneFollowingRate))) {
    //Mark sit fidelity status
    tagStatus = 1;
    //Record tag location
    foodLoc = absLoc;
  }
  //Otherwise
  else {
    //Mark new random location status (may change if we received pheromone location from server)
    tagStatus = 0;
  }
  
  //Adjust location structs
  goalLoc = Ant::Location(Utilities::Polar(nestRadius+collisionDistance,absLoc.pol.theta));
  tempLoc = goalLoc - absLoc;
  
  //Drive to nest
  ant.drive(true);
}
