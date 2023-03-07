/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Lmotor               motor         14              
// Rmotor               motor         17              
// Ltrack               encoder       G, H            
// Rtrack               encoder       A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

float Ltarget = 0;
float Rtarget = 0;

void DriveForward(float _speed, int _Ltarget, int _Rtarget, float distanceGive, float speedGive) {
  float turnKp = 0.4;
  float turnKi = 0.01;
  float turnKd = 0;

  float turnError = 0;
  float turnCumulativeError = 0;
  float turnPreviousError = 0;
  float turnDeltaError = 0;

  float speedKp = 0.1;
  float speedKd = 0.6;

  float speedError = 0;
  float speedPreviousError = 0;
  float speedDeltaError = 0;

  float Lsensor = 0;
  float Rsensor = 0;
  
  float speed = _speed;

  Ltarget += _Ltarget*25.4966;
  Rtarget += _Rtarget*25.4966;
  
  Lmotor.spin(forward);
  Rmotor.spin(forward);

  while ((Lsensor <= Ltarget - (distanceGive*25.4966) || Lsensor >= Ltarget + (distanceGive*25.4966)) || (Rsensor <= Rtarget - (distanceGive*25.4966) || Rsensor >= Rtarget + (distanceGive*25.4966)) || speedDeltaError != 0 +- speedGive) {
    Lmotor.setVelocity(speed, percent); //leader
    Rmotor.setVelocity(speed+(turnKp*turnError)+(turnKi*turnCumulativeError)+(turnKd*turnDeltaError), percent); //follower

    Lsensor = Ltrack.position(degrees);
    Rsensor = Rtrack.position(degrees);

    turnPreviousError = turnError;
    turnError = Lsensor - Rsensor;
    turnCumulativeError += turnError;
    turnDeltaError = turnError-turnPreviousError;

    speedPreviousError = speedError;

    speedError = ((Ltarget-Lsensor)+(Rtarget-Rsensor))/2;
    speedDeltaError = speedError-speedPreviousError;

    speed = (speedKp*speedError)+(speedKd*speedDeltaError);
    
    if (speed > _speed) {
      speed = _speed;
    }
    if (speed < _speed*-1) {
      speed = _speed*-1;
    }

    printf("Applied speed: %f \n", speed);
    printf("Speed from delta error: %f \n", speedDeltaError);
    printf("Ldisp %f \n", Ltarget-Lsensor);
    printf("Rdisp %f \n", Rtarget-Rsensor);
    printf("Displacement %f \n", (Lsensor+Rsensor)*0.0196);
    printf("P Error: %f \n\n", speedError);

    wait(20, msec);
  }

  printf("Done :)\n");

  Lmotor.stop(brake);
  Rmotor.stop(brake);
}

void autonomous(void) {
  Ltrack.setPosition(0, degrees);
  Rtrack.setPosition(0, degrees);

  DriveForward(60, 40, 40, 0.2, 1);
  DriveForward(60, 20, 20, 0.2, 1);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (true) {
    
    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
