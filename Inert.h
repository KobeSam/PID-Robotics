#pragma once
using namespace vex;
#include "robot-config.h"
#include "vex.h"
#include "rightPID.h"
#include "LeftPID.h"

double inertialkP = 0.095;
//was 0.082

double inertialError; 

bool Inertial; 

int inertialDesired;

bool resetInertial;

bool Eheading;

bool ERotation;

int inertialE(){
while(1==1){
while(Inertial) {

    if(Eheading == true){
    inertialError = (inertialDesired - Inertial1.heading(degrees)); 
    }

    if(ERotation == true){
      inertialError = (inertialDesired - Inertial1.rotation(degrees));
    }

    double InMP = inertialError * inertialkP; 

    LeftMotor.spin(forward, InMP, voltageUnits::volt);
    ExtraLeft.spin(forward, InMP, voltageUnits::volt);

    RightMotor.spin(forward, InMP * -1, voltageUnits::volt);
    ExtraRight.spin(forward, InMP * -1, voltageUnits::volt);
  
  }
vex::task::sleep(20);
}
return 1;
}