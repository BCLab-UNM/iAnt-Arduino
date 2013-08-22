/**
 *	Library for all higher-level (requiring multiple base libraries) AntBot functions
 *	on Arduino platform
 **/

#ifndef Ant_h
#define Ant_h

#define ROBOT_INFORMED_NONE 0
#define ROBOT_INFORMED_MEMORY 1
#define ROBOT_INFORMED_PHEROMONE 2

#include <Compass.h>
#include <Movement.h>
#include <Random.h>
#include <SoftwareSerial.h>
#include <Ultrasound.h>
#include <Utilities.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Ant {
public:
    //Structs
    struct Location {
        Utilities util; //object for use in overloaded operators (below)
        Location():pol(Utilities::Polar(0,0)),cart(util.pol2cart(Utilities::Polar(0,0))) {}
        Location(Utilities::Polar p):pol(p),cart(util.pol2cart(p)) {}
        Location(Utilities::Cartesian c):pol(util.cart2pol(c)),cart(c) {}
        Utilities::Polar pol; //polar coordinates (r,theta)
        Utilities::Cartesian cart; //cartesian coordinates (x,y)
        
        //Operator overloading functions:
        //Overload "plus" operator
        Location operator+ (const Location new_location) {
            return Location(util.cart2pol
                            (Utilities::Cartesian(cart.x+new_location.cart.x,cart.y+new_location.cart.y)));
        }
        
        //Overload "minus" operator
        Location operator- (const Location new_location) {
            return Location(util.cart2pol
                            (Utilities::Cartesian(cart.x-new_location.cart.x,cart.y-new_location.cart.y)));
        }
    };
    
    //Legacy Structs
    /*struct Covariance {
     Covariance(){}
     Covariance(float a, float b, float c, float d) {
     matrix[0][0] = a;
     matrix[0][1] = b;
     matrix[1][0] = c;
     matrix[1][1] = d;
     m = n = 2;
     }
     float matrix[2][2];
     int m,n;
     };*/
    
    //Constructors
    Ant();
    Ant(Compass &co,Movement &m,Random &r,SoftwareSerial &sS,Ultrasound &ul,Utilities &ut,
        Location &aL,Location &gL,Location &tL,Utilities::EvolvedParameters &ep,
        unsigned long &gT,const float &nR,const float &rR,const float &cD,
        const float &mR,byte &iS,bool &mC,byte &tSp,byte &rSp,const float &tVe);
    
    //Functions
    void align (float newHeading,int count=1);
    void calibrateCompass();
    void collisionAvoidance(unsigned long &loopTimer);
    void driftCorrection();
    void drive(bool goingHome);
    bool getDirections(long timeout=2000);
    void localize();
    void print(String info="");
    int randomWalk(float fenceRadius);
    int serialFind(String msg, long timeout=2000);
    int serialFind(String msgOne, String msgTwo, long timeout=2000);
    
    //Legacy Functions
    //bool sensorFusion(int n,Location* locs,Covariance* covs,Location &loc_prime,Covariance cov_prime);
    
private:
    //Functions
    byte calibrateSpeed(int degrees);
    int countNeighbors(int tagNum);
    
    //Pointers to objects
    Compass *compass;
    Movement *move;
    Random *randm;
    SoftwareSerial *softwareSerial;
    Ultrasound *us;
    Utilities *util;
    
    //Pointers to structs
    Location *absLoc;
    Location *goalLoc;
    Location *tempLoc;
    Utilities::EvolvedParameters *evolvedParams;
    
    //Pointers to variables
    unsigned long *globalTimer;
    const float *nestRadius;
    const float *robotRadius;
    const float *collisionDistance;
    const float *usMaxRange;
    byte *informedStatus;
    bool *motionCapture;
    byte *travelSpeed;
    byte *rotateSpeed;
    const float *travelVelocity;
};

#endif