#pragma once
using namespace vex;
#include "robot-config.h"
#include "vex.h"

bool ultrasonicEnable;

int ultrasonic(){
  while(1==1){
    while (ultrasonicEnable){
      LeftMotor.setVelocity(20, percent);
      ExtraLeft.setVelocity(20, percent);
      RightMotor.setVelocity(20, percent);
      ExtraRight.setVelocity(20, percent);
      if(LeftSonic.objectDistance(inches) > RightSonic.objectDistance(inches)){
        LeftMotor.spin(forward);
        ExtraLeft.spin(forward);
        RightMotor.spin(reverse);
        ExtraRight.spin(reverse);
      }
      if(RightSonic.objectDistance(inches) > LeftSonic.objectDistance(inches)){
        RightMotor.spin(forward);
        ExtraRight.spin(forward);
        LeftMotor.spin(reverse);
        ExtraLeft.spin(reverse);
      }
    }
    vex::task::sleep(20);
  }
  return 1;
}