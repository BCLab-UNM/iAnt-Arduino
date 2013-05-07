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

//Parameters evolved by GA, initialized to arbitrary starting values
Utilities::EvolvedParameters ep = Utilities::EvolvedParameters(0.05, 0.01, 0.3, 0.3, 1.0, 0.25, 0.25);

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
  
  //Request evolved parameters
  softwareSerial.println("parameters");
  if (ant.serialFind("parameters")) {
    ep.travelGiveUpProbability = softwareSerial.parseFloat();
    softwareSerial.read();
    ep.searchGiveUpProbability = softwareSerial.parseFloat();
    softwareSerial.read();
    ep.uninformedSearchCorrelation = softwareSerial.parseFloat();
    softwareSerial.read();
    ep.informedSearchCorrelationDecayRate = softwareSerial.parseFloat();
    softwareSerial.read();
    ep.stepSizeVariation = softwareSerial.parseFloat();
    softwareSerial.read();
    ep.siteFidelityRate = softwareSerial.parseFloat();
    softwareSerial.read();
    ep.pheromoneFollowingRate = softwareSerial.parseFloat();
    softwareSerial.read();
  }
  
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
 
  bool pheromoneReceived = false;
  //If timeout occurs, we assume no location is available
  if (ant.serialFind("pheromone")) {
    tempLoc.cart.x = softwareSerial.parseInt();
    softwareSerial.read();
    tempLoc.cart.y = softwareSerial.parseInt();
    softwareSerial.read();
    pheromoneReceived = true;
  }
  
  //If no tags were found on the previous trip
  if (tagNeighbors == -1) {
    //We follow pheromones if available
    if (pheromoneReceived) {
      goalLoc = tempLoc;
      tagStatus = 2;
    }
    //Otherwise we choose a new location at random
    else {
      goalLoc = Ant::Location(Utilities::Polar(fenceRadius,randm.boundedUniform(0,359)));
      tagStatus = 0;
    }
  }
  //Otherwise
  else {
    //We follow pheromones probabilistically if available
    if (pheromoneReceived && (randm.uniform() < util.exponentialCDF(9 - tagNeighbors, ep.pheromoneFollowingRate)) && (randm.uniform() > util.exponentialCDF(tagNeighbors + 1, ep.siteFidelityRate))) {
      goalLoc = tempLoc;
      tagStatus = 2;
    }
    //Or we use site fidelity probabilistically
    else if ((randm.uniform() < util.exponentialCDF(tagNeighbors + 1, ep.siteFidelityRate)) && (!pheromoneReceived || (randm.uniform() > util.exponentialCDF(9 - tagNeighbors, ep.pheromoneFollowingRate)))) {
      goalLoc = foodLoc;
      tagStatus = 1;
    }
    //If neither pheromones nor site fidelity, we choose a new location at random
    else {
      goalLoc = Ant::Location(Utilities::Polar(fenceRadius,randm.boundedUniform(0,359)));
      tagStatus = 0;
    }
  }
  
  //Calculate directions leading from current position to goal
  tempLoc = goalLoc - absLoc;
  
  //Drive to goal
  ant.drive(false);
  
  //Perform random walk with varying turn radius depending on whether food was found on previous trip
  tagNeighbors = ant.randomWalk(fenceRadius);
  
  //If a tag was found, we record its location
  if (tagNeighbors >= 0) {
    foodLoc = absLoc;
  }
  
  //Adjust location structs
  goalLoc = Ant::Location(Utilities::Polar(nestRadius+collisionDistance,absLoc.pol.theta));
  tempLoc = goalLoc - absLoc;
  
  //Drive to nest
  ant.drive(true);
}
