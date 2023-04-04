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
// TurnTrack            encoder       E, F            
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

float YawTarget = 0;

float inToDeg = 1;
float TrackDegToYawDeg = 1;

float turnKp = 0.3;
float turnKi = 0;
float turnKd = 0;

float distanceKp = 0.3;
float distanceKd = 0.6;

float yawKp = 0;
float yawKd = 0;

float pointKp = 0;
float pointKi = 0;
float pointKd = 0;

void ResetTranslationalDisplacement() {
  Ltarget = 0;
  Rtarget = 0;

  Ltrack.setPosition(0, degrees);
  Rtrack.setPosition(0, degrees);
}

void ResetAngularDisplacement() {
  TurnTrack.setPosition(0, degrees);
}

void SetProportionOfInchesToDegrees(float _conversionFactor) {
  inToDeg = _conversionFactor;
}

void SetProportionOfTrackingWheelDegreesToRobotYawDegrees(float _conversionFactor) {
  TrackDegToYawDeg = _conversionFactor;
}

void SetDrivingKs(float _turnKp, float _turnKi, float _turnKd, float _distanceKp, float _distanceKd) {
  turnKp = _turnKp;
  turnKi = _turnKi;
  turnKd = _turnKd;

  distanceKp = _distanceKp;
  distanceKd = _distanceKd;
}

void SetTurningKs(float _pointKp, float _pointKi, float _pointKd, float _yawKp, float _yawKd) {
  pointKp = _pointKp;
  pointKi = _pointKi;
  pointKd = _pointKd;

  yawKp = _yawKp;
  yawKd = _yawKd;
}

void DriveForward(float _speed, float _target, float distanceGive, float speedGive) {

  float turnError = 0;
  float turnCumulativeError = 0;
  float turnPreviousError = 0;
  float turnDeltaError = 0;

  float distanceError = 0;
  float distancePreviousError = 0;
  float distanceDeltaError = 0;

  float Lsensor = 0;
  float Rsensor = 0;
  
  float speed = _speed;

  Ltarget += _target*inToDeg;
  Rtarget += _target*inToDeg;
  
  Lmotor.spin(forward);
  Rmotor.spin(forward);

  while (Ltarget-Lsensor > distanceGive*inToDeg || Ltarget-Lsensor < distanceGive*inToDeg*-1 || Rtarget-Rsensor > distanceGive*inToDeg || Rtarget-Rsensor < distanceGive*inToDeg*-1 || distanceDeltaError != 0 +- speedGive) {
    Lmotor.setVelocity(speed, percent); //leader
    Rmotor.setVelocity(speed+(turnKp*turnError)+(turnKi*turnCumulativeError)+(turnKd*turnDeltaError), percent); //follower

    Lsensor = Ltrack.position(degrees);
    Rsensor = Rtrack.position(degrees);

    turnPreviousError = turnError;
    turnError = Lsensor - Rsensor;
    turnCumulativeError += turnError;
    turnDeltaError = turnError-turnPreviousError;

    distancePreviousError = distanceError;

    distanceError = Ltarget-Lsensor;
    distanceDeltaError = distanceError-distancePreviousError;

    speed = (distanceKp*distanceError)+(distanceKd*distanceDeltaError);
    
    if (speed > _speed) {
      speed = _speed;
    }
    if (speed < _speed*-1) {
      speed = _speed*-1;
    }

    printf("Applied speed: %f \n", speed);
    printf("Speed from delta error: %f \n", distanceDeltaError);// ETHAN LEFT HIS COMPUTER OPEN. 
    printf("Ldisp %f \n", (Ltarget-Lsensor)*(1/inToDeg));// IT IS CURRENTLY 3:20 AND HE HAS GONE HOME
    printf("Rdisp %f \n", (Rtarget-Rsensor)*(1/inToDeg));//DAVID DORLAND DID NOT CHANGE ANYTHING EXCEPT THESE COMMENTS 
    printf("Displacement %f \n", (Lsensor+Rsensor)*(0.5/inToDeg));//GOOD CODE
    printf("P Error: %f \n\n", distanceError);

    printf("Ltrack: %f \n\n", Ltrack.position(degrees));

    wait(20, msec);
  }

  printf("Done :)\n");

  Lmotor.stop(brake);
  Rmotor.stop(brake);
}

void TurnTo(float _speed, float _target, float degreeGive, float angularSpeedGive) {
  float tempTarget = _target;
  float currentAngle = TurnTrack.position(degrees)*-1;

  if (TurnTrack.position(degrees) >= TrackDegToYawDeg*360) {
    TurnTrack.setPosition(TurnTrack.position(degrees)-(TrackDegToYawDeg*360), degrees);
  }
  else if (TurnTrack.position(degrees) < 0) {
    TurnTrack.setPosition(TurnTrack.position(degrees)+(TrackDegToYawDeg*360), degrees);
  }

  while (tempTarget < 0 || tempTarget >= 360) {
    if (tempTarget < 0) {
      tempTarget += 360;
    }
    else {
      tempTarget -= 360;
    }
  }

  YawTarget = tempTarget*TrackDegToYawDeg;

  while (YawTarget < 0 || YawTarget >= TrackDegToYawDeg*360) {
    if (YawTarget < 0) {
      YawTarget += TrackDegToYawDeg*360;
    }
    else {
      YawTarget -= TrackDegToYawDeg*360;
    }
  }

  float tempRelativeTarget = YawTarget - currentAngle;

  float speed = _speed;
  float turnError = 0;
  float turnPreviousError = 0;
  float turnDeltaError = turnError - turnPreviousError;

  while (tempRelativeTarget < 0 || tempRelativeTarget >= TrackDegToYawDeg*360) {
    if (tempRelativeTarget < 0) {
      tempRelativeTarget += TrackDegToYawDeg*360;
    }
    else {
      tempRelativeTarget -= TrackDegToYawDeg*360;
    }
  }

  Lmotor.spin(forward);
  Rmotor.spin(forward);

  if (tempRelativeTarget < TrackDegToYawDeg*180) {
    printf("turning right\n");
    //turn right
  }
  else {
    printf("turning left\n");
    //turn left
    YawTarget -= TrackDegToYawDeg*360;
  }

  while (YawTarget - currentAngle > degreeGive*TrackDegToYawDeg || YawTarget - currentAngle < degreeGive*TrackDegToYawDeg*-1 || turnDeltaError > angularSpeedGive || turnDeltaError < angularSpeedGive*-1) {
    Lmotor.setVelocity(speed, percent);
    Rmotor.setVelocity(speed*-1, percent);
    currentAngle = TurnTrack.position(degrees)*-1;

    turnError = YawTarget - currentAngle;
    turnDeltaError = turnError - turnPreviousError;

    speed = (yawKp*turnError)+(yawKd*turnDeltaError);

    if (speed <= _speed*-1) {
      speed = _speed*-1;
    }
    if (speed >= _speed) {
      speed = _speed;
    }

    turnPreviousError = turnError;

    wait(20, msec);

    printf("Degrees remaining: %f\n", turnError);
    printf("distance give: %f\n", degreeGive*TrackDegToYawDeg);
    printf("Actual speed: %f\n", turnDeltaError);
    printf("angular speed give: %f\n\n", angularSpeedGive);
  }

  Lmotor.stop(brake);
  Rmotor.stop(brake);

  printf("done :)\n");
}

void TurnRight(float _speed, float _target, float degreeGive, float angularSpeedGive) {

  float speed = _speed;

  float yawError = 0;
  float yawPreviousError = 0;
  float yawDeltaError = 0;

  float pointError = 0;
  float pointCumulativeError = 0;
  float pointPreviousError = 0;
  float pointDeltaError = 0;

  float Lsensor = 0;
  float Rsensor = 0;

  Ltarget += _target*TrackDegToYawDeg;
  Rtarget -= _target*TrackDegToYawDeg;
  
  Lmotor.spin(forward);
  Rmotor.spin(forward);

  while (Ltarget-Lsensor > degreeGive*TrackDegToYawDeg || Ltarget-Lsensor < degreeGive*TrackDegToYawDeg*-1 || Rtarget-Rsensor > degreeGive*TrackDegToYawDeg || Rtarget-Rsensor < degreeGive*TrackDegToYawDeg*-1 || yawDeltaError != 0 +- degreeGive) {
    Lmotor.setVelocity(speed, percent); //leader
    Rmotor.setVelocity((speed+(pointKp*pointError)+(pointKi*pointCumulativeError)+(pointKd*pointDeltaError))*-1, percent); //follower

    Lsensor = Ltrack.position(degrees);
    Rsensor = Rtrack.position(degrees);

    pointPreviousError = pointError;
    pointError = Lsensor + Rsensor;
    pointCumulativeError += pointError;
    pointDeltaError = pointError-pointPreviousError;

    yawPreviousError = yawError;

    yawError = Ltarget-Lsensor;
    yawDeltaError = yawError-yawPreviousError;

    speed = (distanceKp*yawError)+(distanceKd*yawDeltaError);
    
    if (speed > _speed) {
      speed = _speed;
    }
    if (speed < _speed*-1) {
      speed = _speed*-1;
    }

    printf("Applied speed: %f \n", speed);
    printf("Speed from delta error: %f \n", yawDeltaError);
    printf("Ldisp %f \n", (Ltarget-Lsensor)*(1/TrackDegToYawDeg));
    printf("Rdisp %f \n", (Rtarget-Rsensor)*(1/TrackDegToYawDeg));
    printf("Point error %f \n", pointError);
    printf("P Error: %f \n\n", yawError);

    printf("Ltrack: %f \n\n", Ltrack.position(degrees));

    wait(20, msec);
  }

  printf("Done :)\n");

  Lmotor.stop(brake);
  Rmotor.stop(brake);
}

void autonomous(void) {
  Ltrack.setPosition(0, degrees);
  Rtrack.setPosition(0, degrees);

  SetProportionOfInchesToDegrees(27.2);
  SetProportionOfTrackingWheelDegreesToRobotYawDegrees(3.18); //used to be 4.65 when the back tracking wheel was used

  SetDrivingKs(0.3, 0, 0, 0.3, 0.6);
  SetTurningKs(0.5, 0, 0, 0.3, 0.5);

  //DriveForward(50, 20, 0.2, 0.9);
  TurnRight(40, 360, 1, 1);
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
    printf("tdeg: %f\n", TurnTrack.position(degrees));
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
