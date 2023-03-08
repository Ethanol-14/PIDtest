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

float inToDeg = 27.2;

void DriveForward(float _speed, float _target, float distanceGive, float speedGive) {
  float turnKp = 0.3;
  float turnKi = 0;
  float turnKd = 0;

  float turnError = 0;
  float turnCumulativeError = 0;
  float turnPreviousError = 0;
  float turnDeltaError = 0;

  float speedKp = 0.3;
  float speedKd = 0.6;

  float speedError = 0;
  float speedPreviousError = 0;
  float speedDeltaError = 0;

  float Lsensor = 0;
  float Rsensor = 0;
  
  float speed = _speed;

  Ltarget += _target*inToDeg;
  Rtarget += _target*inToDeg;
  
  Lmotor.spin(forward);
  Rmotor.spin(forward);

  while ((Lsensor <= Ltarget - (distanceGive*inToDeg) || Lsensor >= Ltarget + (distanceGive*inToDeg)) || (Rsensor <= Rtarget - (distanceGive*inToDeg) || Rsensor >= Rtarget + (distanceGive*inToDeg)) || speedDeltaError != 0 +- speedGive) {
    Lmotor.setVelocity(speed, percent); //leader
    Rmotor.setVelocity(speed+(turnKp*turnError)+(turnKi*turnCumulativeError)+(turnKd*turnDeltaError), percent); //follower

    Lsensor = Ltrack.position(degrees);
    Rsensor = Rtrack.position(degrees);

    turnPreviousError = turnError;
    turnError = Lsensor - Rsensor;
    turnCumulativeError += turnError;
    turnDeltaError = turnError-turnPreviousError;

    speedPreviousError = speedError;

    speedError = Ltarget-Lsensor;
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
    printf("Displacement %f \n", (Lsensor+Rsensor)*0.0368);
    printf("P Error: %f \n\n", speedError);

    wait(20, msec);
  }

  printf("Done :)\n");

  Lmotor.stop(brake);
  Rmotor.stop(brake);
}

void TurnTo(float _speed, float target, float degreeGive, float angularSpeedGive) {

}

void autonomous(void) {
  Ltrack.setPosition(0, degrees);
  Rtrack.setPosition(0, degrees);

  DriveForward(50, 40, 0.2, 2);
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
