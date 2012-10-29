//////////////////////////
////Required Libraries////
//////////////////////////

//Built-in libraries
#include <SoftwareSerial.h>
#include <Wire.h> //Interface for compass

//Libraries acquired from elsewhere
#include <PID_v1.h>
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

//Food
bool tagFound = false; //indicates whether tag has been found while searching
int tagNeighbors = -1; //holds number of neighboring tags found near tag (negative value indicates no tag found)

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

//SoftwareSerial
const byte ssRx = 4;
const byte ssTx = 5;

//Ultrasound
const byte usEcho = 16; //Ultrasonic echo pin
const byte usTrigger = 17; //Ultrasonic trigger pin
const byte usMaxRange = 300; //limit of ultrasound (we ignore any values returned above this threshold)
const float collisionDistance = 30; //threshold distance for collision recognition (cm)
const float nestRadius = 8; //radius of the nest (cm)

//Other
unsigned long globalTimer; //holds start time of current run

////////////////////////////
////Class Instantiations////
////////////////////////////

SoftwareSerial softwareSerial(ssRx,ssTx);
Utilities util;
Compass compass = Compass(util);
Movement move = Movement(speed_right,speed_left,dir_right,dir_left,simFlag);
Ultrasound us = Ultrasound(usTrigger,usEcho,simFlag,usMaxRange);
Random randm;
Ant ant = Ant(compass,move,softwareSerial,us,util,absLoc,goalLoc,tempLoc,globalTimer,nestRadius,collisionDistance,usMaxRange);


/////////////
////Setup////
/////////////

void setup()
{
  //Open local serial connection for debugging
  Serial.begin(9600);
 
  //Open serial connection to iDevice
  softwareSerial.begin(9600);

  //Request random seed
  softwareSerial.println("seed");
  ant.serialFind("seed");
  
  //Start prng with received value
  randomSeed(softwareSerial.parseInt());

  //Start global timer
  globalTimer = micros();
  
  //Toss first compass reading to ensure correct values elsewhere
  compass.heading();

  //Localize to find starting point
  absLoc = ant.localize(70);
}

/////////////////
////Main Loop////
/////////////////

void loop()
{
  //We use four location structs:
  //1. goalLoc will hold the location of the goal in relation to the nest (NOTE: A new goal is randomly selected here if food was not found in last search)
  if (!tagFound) goalLoc = Ant::Location(Utilities::Polar(randm.boundedUniform(nestRadius+collisionDistance,fenceRadius),randm.boundedUniform(0,359)));
  else goalLoc = foodLoc;
  //2. tempLoc holds distance and heading from the *start* of the current leg to the goal
  tempLoc = goalLoc - absLoc;
  //3. absLoc holds the location of the robot relative to the nest
  //4. foodLoc holds the location of any food discovered while searching
  
  //Dump to ABS
  ant.print();
    
  //Drive to goal
  ant.drive(120);
  
  //Perform random walk with varying turn radius depending on whether food was found on previous trip
  tagNeighbors = ant.randomWalk(randm,60,22.5,fenceRadius,tagFound);
  
  //If tagNeighbors is 0 or more
  if (tagNeighbors >= 0) {
    //Then at least one tag was found
    tagFound = true;
  }
  //Otherwise
  else {
    //No tags were found
    tagFound = false;
  }
  
  //Adjust location structs
  goalLoc = Ant::Location(Utilities::Polar(nestRadius+collisionDistance,absLoc.pol.theta));
  tempLoc = goalLoc - absLoc;
  
  //If food was found
  if (tagFound) {
    //Record its location
    foodLoc = absLoc;
  }
  
  //Drive to nest
  ant.drive(120);
  
  //Signal ABS via iDevice of arrival at nest
  ant.print("home");
  
  //Request virtual pheromone location from iDevice
  softwareSerial.println("pheromone on");
 
  if (tagNeighbors <= 0)
  {
    //Check for new virtual pheromone location from iDevice
    //If timeout occurs, we assume no location is available
    if (ant.serialFind("pheromone"))
    {
      foodLoc.cart.x = softwareSerial.parseInt();
      softwareSerial.read();
      foodLoc.cart.y = softwareSerial.parseInt();
      tagFound = true;
    }
  }
  
  //Signal iDevice to disable pheromone check
  softwareSerial.println("pheromone off");
}
