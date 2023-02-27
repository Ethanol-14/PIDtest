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

void driveTo(float _speed, int _Ltarget, int _Rtarget) {
  float Kp = 0.3;
  float Ki = 0;
  float Kd = 0;

  float error = 0;
  float cumulativeError = 0;
  float previousError = 0;
  float deltaError = 0;

  float Lsensor = 0;
  float Rsensor = 0;

  float speed = _speed;
  float Ltarget = _Ltarget*28.6479;
  float Rtarget = _Rtarget*28.6479;
  
  Lmotor.spin(forward);
  Rmotor.spin(forward);

  while (Lsensor < Ltarget || Rsensor < Rtarget) {
    Lmotor.setVelocity(speed, percent); //leader
    Rmotor.setVelocity(speed+(Kp*error)+(Ki*cumulativeError)+(Kd*deltaError), percent); //follower

    Lsensor = Ltrack.position(degrees);
    Rsensor = Rtrack.position(degrees);

    previousError = error;
    error = Lsensor - Rsensor;
    cumulativeError += error;
    deltaError = error-previousError;

    wait(20, msec);
  }

  Lmotor.stop(brake);
  Rmotor.stop(brake);
}

void autonomous(void) {
  Ltrack.setPosition(0, degrees);
  Rtrack.setPosition(0, degrees);

  driveTo(50, 20, 20);
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
