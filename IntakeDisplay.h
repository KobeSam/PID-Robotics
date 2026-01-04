#pragma once
using namespace vex;
#include "robot-config.h"
#include "vex.h"

int intakeSpeed(){
while(1){
  Controller2.Screen.clearLine();
  Controller2.Screen.print(round(intake.velocity(percent)));
  vex::task::sleep(50);
}
}