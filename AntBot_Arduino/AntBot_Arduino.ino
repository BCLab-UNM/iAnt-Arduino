/////////////////////////
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
Utilities::EvolvedParameters ep = Utilities::EvolvedParameters(0.05, 0.01, 0.3, 0.3, 1.0, 4.0);

//Food
byte informedStatus = 0; //Indicates what type of information is influencing the robot's behavior (ROBOT_INFORMED_NONE, ROBOT_INFORMED_MEMORY, or ROBOT_INFORMED_PHEROMONE)
int tagsFound = 0; //holds number of tags found

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
byte rotateSpeed = 140; //motor speed used while rotating left/right

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
Ant ant = Ant(compass,move,randm,softwareSerial,us,util,absLoc,goalLoc,tempLoc,ep,globalTimer,nestRadius,robotRadius,collisionDistance,usMaxRange,informedStatus,mocapFlag,travelSpeed,rotateSpeed,travelVelocity);


/////////////
////Setup////
/////////////

String message;

void setup()
{
  softwareSerial.begin(9600);
  softwareSerial.println("ready");
  message = "";
}

/////////////////
////Main Loop////
/////////////////

void loop() {
  if(softwareSerial.available()) {
    char c = softwareSerial.read();
  
    if(c == ',' || c == '\n') {
      parse();
      message = "";
    }
    else if(c != -1) {
      message += c;
    }
  }
}

void parse() {
  if(message == "motors") {
    int rotate = softwareSerial.parseInt();
    int forward = softwareSerial.parseInt();
    ant.adjustMotors(rotate, forward);
  }
  else if(message == "drive") {
    float distance = softwareSerial.parseFloat();
    ant.drive(distance);
    softwareSerial.println("drive");
  }
  else if(message == "align") {
    float heading = softwareSerial.parseFloat();
    ant.align(heading);
    softwareSerial.println("align");
  }
  else if(message == "compass") {
    float heading = ant.getCompass();
    char str[10];
    dtostrf(heading, 6, 2, str);
    softwareSerial.println("compass," + String(str));
  }
  else if(message == "ultrasound") {
    float distance = ant.getUltrasound();
    char str[10];
    dtostrf(distance, 6, 2, str);
    softwareSerial.println("ultrasound," + String(str));
  }
}
