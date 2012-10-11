//////////////////////////
////Required Libraries////
//////////////////////////

//Built-in libraries
#include <SoftwareSerial.h>
#include <Wire.h> //Interface for compass

//Libraries acquired from elsewhere
#include <MatrixMath.h>
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
bool simFlag = true;

//Food
bool tagFound = false; //indicates whether tag has been found while searching?
int tagNeighbors = -1; //holds number of neighboring tags found near tag (negative value indicates no tag found)

//Location
Ant::Location absLoc; //holds absolute location (relative to nest)
Ant::Location goalLoc; //holds current goal location
Ant::Location tempLoc; //holds current location (relative to current leg)
Ant::Location foodLoc; //holds location of last food found
const float fenceRadius = 300; //radius of virtual fence (cm)

//PID controller
double input, output, setpoint;
PID pid(&input,&output,&setpoint,2,5,1,DIRECT);

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
const float collisionDistance = 30; //threshold distance for collision recognition (cm)
const float nestRadius = 8; //radius of the nest (cm)
Ant::Covariance usCov = Ant::Covariance(1.9390,-0.5570,-0.5570,0.3142); //ultrasound covariance matrix

//Other
unsigned long globalTimer; //holds start time of current run

////////////////////////////
////Class Instantiations////
////////////////////////////

SoftwareSerial softwareSerial(ssRx,ssTx);
Utilities util;
Compass compass = Compass(util);
Movement move = Movement(speed_right,speed_left,dir_right,dir_left,simFlag);
Ultrasound us = Ultrasound(usTrigger,usEcho,simFlag);
Random randm;
Ant ant = Ant(compass,move,softwareSerial,us,util,absLoc,goalLoc,tempLoc,globalTimer,nestRadius);


/////////////
////Setup////
/////////////

void setup()
{
  //Open local serial connection for debugging
  Serial.begin(9600);

  //Open serial connection to iDevice
  softwareSerial.begin(9600);
  
  delay(1000);
  
  //Request randm seed
  softwareSerial.println("seed");
  while (!ant.serialFind("seed")) {}
  
  //Start prng with received value
  randomSeed(softwareSerial.parseInt());

  //Start global timer
  globalTimer = micros();
  
  //Toss first compass reading to ensure correct values elsewhere
  compass.heading();
  
  //Start PID controller
  pid.SetMode(AUTOMATIC);
  //Initialize settings
  pid.SetOutputLimits(-127,127);
  pid.SetSampleTime(50); //ms
  
  //Localize to find starting point
  absLoc = ant.localize(70);
}

/////////////////
////Main Loop////
/////////////////

void loop()
{
  //We use four location structs:
  //1. goalLoc will hold the location of the goal in relation to the nest (NOTE: A new goal is randmly selected here if food was not found in last search)
  if (!tagFound) goalLoc = Ant::Location(Utilities::Polar(randm.boundedUniform(nestRadius+collisionDistance,200),randm.boundedUniform(0,359)));
  else goalLoc = foodLoc;
  //2. temLoc holds distance and heading from the *start* of the current leg to the goal
  tempLoc = goalLoc - absLoc;
  //3. absLoc holds the location of the robot relative to the net
  //4. foodLoc holds the location of any food discovered while searching
  
  //Dump to ABS
  ant.print();

  //Align to heading
  ant.align(tempLoc.pol.theta,70,50);
  
  //legTimer is used to measure distance of entire segment
  //loopTimer is used to measure distance covered during each iteration of while loop
  unsigned long loopTimer;
  
  //Ask iDevice to enable gyroscope
  softwareSerial.println("gyro on");
  while (!ant.serialFind("gyro on")) {}
  
  loopTimer = millis();
  setpoint = 0;
  //**Assume fixed velocity of 37.7 cm/s**
  util.tic(tempLoc.pol.r/37.7*1000);
  //Tasks:
  //1. Check for obstacles within range; avoid if necessary
  //2. Perform drift correction
  while (!util.isTime())
  {
    ant.collisionAvoidance(collisionDistance,120,loopTimer); //check for collision; maneuver and update location if needed
    ant.driftCorrection(pid,input,output,120); //correct for motor drift if needed
  }
  move.stopMove();

  //Ask iDevice to disable gyroscope
  softwareSerial.println("gyro off");
  
  //Update absolute location
  absLoc = goalLoc;
  
  //Perform randm walk with varying turn radius depending on whether food was found on previous trip
  tagNeighbors = ant.randomWalk(randm,60,22.5,collisionDistance,fenceRadius,tagFound);
  if (tagNeighbors < 0) tagFound = false;
  else tagFound = true;
  
  //Adjust location structs
  goalLoc = Ant::Location(Utilities::Polar(nestRadius+collisionDistance,absLoc.pol.theta));
  tempLoc = goalLoc - absLoc;
  
  //If food was found, replace goal location with current absolute location and light green LED
  if (tagFound) foodLoc = absLoc; //record location
 
  //Align to heading
  ant.align(tempLoc.pol.theta,70,50);

  //Ask iDevice to enable gyroscope
  softwareSerial.println("gyro on");
  while (!ant.serialFind("gyro on")) {}
  
  loopTimer = millis();
  setpoint = 0;
  util.tic(tempLoc.pol.r/37.7*1000);
  //Tasks:
  //1. Check for obstacles within range; avoid if necessary
  //2. Perform drift correction
  //**Assume fixed velocity of 37.7 cm/s**
  while (!util.isTime())
  {
    ant.collisionAvoidance(collisionDistance,120,loopTimer); //check for collision; maneuver and update location if needed
    ant.driftCorrection(pid,input,output,120); //correct for motor drift if needed
  }
  move.stopMove();

  //Ask iDevice to disable gyroscope
  softwareSerial.println("gyro off");
  
  //Replace absolute location with final location
  absLoc = goalLoc;
  
  //Signal ABS of arrival at nest
  ant.print("home");
  
  //Request virtual pheromone location from iDevice
  softwareSerial.println("pheromone on");
 
  if (tagNeighbors <= 0)
  {
    //Check for new virtual pheromone location from iDevice
    //If timeout occurs (1000 ms), we assume no location is available
    if (ant.serialFind("pheromone"))
    {
      foodLoc.cart.x = softwareSerial.parseInt();
      softwareSerial.read();
      foodLoc.cart.y = softwareSerial.parseInt();
    }
  }
  
  //Signal iDevice to disable pheromone check
  softwareSerial.println("pheromone off");
}
