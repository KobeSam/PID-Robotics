#pragma once
using namespace vex;
#include "robot-config.h"
#include "vex.h"

double rightkP = -0.02; //1 tune until steady minor issolation 

// tune normally has large sig-figs

int rightDesiredValue;


// lateral PID
int rightError = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////

bool rightResetDriveSensors = false; 

//variable modified for use 
bool rightEnableDrivePID = true; 
//bool ITrue = true;

int rightDrivePID(){
while(1==1){
while(rightEnableDrivePID) {
  if (rightResetDriveSensors) {
    rightResetDriveSensors = false; 

    RightMotor.setPosition(0,degrees);

  }

  // get position of both motors
  int rightMotorPosition = RightMotor.position(degrees);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Lateral Movement PID
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // get the mean of the left and right motor (2m drive)
  int rightAveragePosition = rightMotorPosition;

  //Potential
  rightError = rightAveragePosition - rightDesiredValue; 
  //IError = Inertial.heading() - IDV;

  double rightLateralMotorPower = (rightError * rightkP); // can /12
  //double IPW = (IError * IKP);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  RightMotor.spin(forward, rightLateralMotorPower, voltageUnits::volt);
  ExtraRight.spin(forward, rightLateralMotorPower, voltageUnits::volt);
  
  }

vex::task::sleep(20);
}
return 1;
}

