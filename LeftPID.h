#pragma once
using namespace vex;
#include "robot-config.h"
#include "vex.h"

double leftkP = -0.02; //1 tune until steady minor issolation 

// tune normally has large sig-figs

int leftDesiredValue; 


// lateral PID
int leftError = 0; // sensor value (current) - desired value // (positional value) deltaX // sensorValue - DesiredValue : Position
////////////////////////////////////////////////////////////////////////////////////////////////////////

bool leftResetDriveSensors = false; 

//variable modified for use 
bool leftEnableDrivePID = true; 

int leftDrivePID(){
while(1==1){
while(leftEnableDrivePID) {
  if (leftResetDriveSensors) {
    leftResetDriveSensors = false; 

    LeftMotor.setPosition(0,degrees);

  }

  // get position of both motors
  int leftMotorPosition = LeftMotor.position(degrees);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Lateral Movement PID
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // get the mean of the left and right motor (2m drive)
  int leftAveragePosition = leftMotorPosition;

  //Potential
  leftError = leftAveragePosition - leftDesiredValue; 

  double leftLateralMotorPower = (leftError * leftkP); // can /12
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  LeftMotor.spin(forward, leftLateralMotorPower, voltageUnits::volt);
  ExtraLeft.spin(forward, leftLateralMotorPower, voltageUnits::volt);
  
  }

vex::task::sleep(20);
}
return 1;
}

