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
// intake               motor         15              
// POut                 digital_out   A               
// Controller2          controller                    
// Vision1              vision        21              
// Arm                  motor         7               
// ExtraRight           motor         5               
// ExtraLeft            motor         3               
// BackLift1            digital_out   B               
// BackLift2            digital_out   C               
// RightMotor           motor_group   8, 4            
// LeftMotor            motor_group   13, 6           
// Inertial1            inertial      14              
// LeftSonic            distance      16              
// RightSonic           distance      2               
// Cover                digital_out   D               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

#include "LeftPID.h"
#include "RightPID.h"

#include "VisionRightPID.h"
#include "VisionLeftPID.h"

#include "IntakeDisplay.h"

#include "Inert.h"

#include "UltraSonic.h"

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

  leftEnableDrivePID = false;
  rightEnableDrivePID = false;
  rightEnableVisionPID = false;
  leftEnableVisionPID = false;

  Arm.setVelocity(100, percent);
  LeftMotor.setVelocity(100, percent);
  RightMotor.setVelocity(100, percent);
  ExtraRight.setVelocity(100, percent);
  ExtraLeft.setVelocity(100, percent);
  intake.setVelocity(100, percent);

  Arm.setStopping(hold);
  LeftMotor.setStopping(hold);
  RightMotor.setStopping(hold);
  ExtraLeft.setStopping(hold);
  ExtraRight.setStopping(hold);

  Arm.setPosition(0, degrees);

  BackLift1.set(false);
  BackLift2.set(false);

  POut.set(false);

  Cover.set(false);

  Inertial1.setHeading(90, degrees);
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

void turnP(double targetAngle){
  double kP = 0.1;
  double kD = 0.5;
  double error;
  double tpower;
  double derivative;
  double prevError;
  double tdpower;

  imu.resetRotation(); // show difference between IMU rotation and heading, make sure we calbirate IMU, and calibrate heading to 0

  while (true) {
    double currentAngle = imu.rotation(degrees); // show difference between IMU rotation and heading
    error = targetAngle - currentAngle;
    tpower = kP * error;
    derivative = error - prevError;
    tdpower = kD * derivative;

    LeftMotor.spin(forward, tpower + tdpower, voltageUnits::volt);
    ExtraLeft.spin(forward, tpower, voltageUnits::volt);
    RightMotor.spin(forward, -tpower, voltageUnits::volt);
    ExtraRight.spin(forward, -tpower, voltageUnits::volt);

    prevError = error;

    if (fabs(error) < 0.4){
      break;
    }
    vex::task::sleep(20);
  }

}

void autonomous(void) {
  vex::task randomname(leftDrivePID);
  vex::task randoname(rightDrivePID);
  vex::task randname(rightVisionPID);
  vex::task ranname(leftVisionPID);
  vex::task Iasdsad(inertialE);
  vex::task asdsadasd(ultrasonic);

  Inertial1.setHeading(90, degrees);

  Inertial = false;

  ultrasonicEnable = false;

  Arm.spinToPosition(200, degrees);

  rightEnableDrivePID = true;
  leftEnableDrivePID = true;
  rightResetDriveSensors = true;
  leftResetDriveSensors = true;
  rightDesiredValue = 400;
  leftDesiredValue = 400;
  vex::task::sleep(550);

  POut.set(true);

  wait(0.2, seconds);

  rightEnableDrivePID = true;
  leftEnableDrivePID = true;
  rightResetDriveSensors = true;
  leftResetDriveSensors = true;
  rightDesiredValue = -300;
  leftDesiredValue = -300;
  wait(0.4, seconds);
  Arm.spinToPosition(-50, degrees, false);
  vex::task::sleep(450);

  rightEnableDrivePID = false;
  leftEnableDrivePID = false;
  Inertial = true;
  Eheading = true;
  inertialkP = 0.1;
  inertialDesired = 0;
  vex::task::sleep(1500);
  Eheading = false;
  Inertial = false;

  rightEnableDrivePID = true;
  leftEnableDrivePID = true;
  rightResetDriveSensors = true;
  leftResetDriveSensors = true;
  rightDesiredValue = 500;
  leftDesiredValue = 500;
  vex::task::sleep(650);

  rightEnableDrivePID = false;
  leftEnableDrivePID = false;
  Inertial = true;
  Eheading = true;
  inertialkP = 0.1;
  inertialDesired = 98;
  vex::task::sleep(1500);
  Eheading = false;
  Inertial = false;  
  
  rightEnableDrivePID = true;
  leftEnableDrivePID = true;
  rightResetDriveSensors = true;
  leftResetDriveSensors = true;
  rightDesiredValue = 2400;
  leftDesiredValue = 2400;
  vex::task::sleep(2550);

  rightEnableDrivePID = true;
  leftEnableDrivePID = true;
  rightResetDriveSensors = true;
  leftResetDriveSensors = true;
  rightDesiredValue = -350;
  leftDesiredValue = -350;
  vex::task::sleep(500);

  rightEnableDrivePID = false;
  leftEnableDrivePID = false;
  Inertial = true;
  ERotation = true;
  inertialkP = 0.1;
  inertialDesired = 180;
  vex::task::sleep(1500);
  ERotation = false;
  Inertial = false; 

  rightEnableDrivePID = true;
  leftEnableDrivePID = true;
  rightResetDriveSensors = true;
  leftResetDriveSensors = true;
  rightDesiredValue = -500;
  leftDesiredValue = -500;
  vex::task::sleep(650);

  BackLift1.set(true);
  BackLift2.set(true);

  wait(0.4, seconds);

  rightEnableDrivePID = true;
  leftEnableDrivePID = true;
  rightResetDriveSensors = true;
  leftResetDriveSensors = true;
  rightDesiredValue = 400;
  leftDesiredValue = 400;
  Arm.spinToPosition(200, degrees, false);
  intake.spin(reverse);
  vex::task::sleep(550);

  rightEnableDrivePID = false;
  leftEnableDrivePID = false;
  Inertial = true;
  Eheading = true;
  inertialkP = 0.1;
  inertialDesired = 1;
  Arm.spinToPosition(0, degrees, false);
  vex::task::sleep(1500);
  Eheading = false;
  Inertial = false; 

  rightEnableDrivePID = true;
  leftEnableDrivePID = true;
  rightResetDriveSensors = true;
  leftResetDriveSensors = true;
  rightDesiredValue = 800;
  leftDesiredValue = 800; 
  vex::task::sleep(950);

  POut.set(false);

  rightEnableDrivePID = true;
  leftEnableDrivePID = true;
  rightResetDriveSensors = true;
  leftResetDriveSensors = true;
  rightDesiredValue = -900;
  leftDesiredValue = -900;
  vex::task::sleep(1050);

  BackLift1.set(false);
  BackLift2.set(false);




  


  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                  
            User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void PneumaticOut(){
      POut.set(false);
    }

void PneumaticIn(){
   POut.set(true);
 }
 
void usercontrol(void) {
  int deadband = 5;

  vex::task raname(intakeSpeed);


  
  // User control code here, inside the loop

  intake.setVelocity(75, percent);
  
  while (1) {
    int RightStick = Controller1. Axis3. position() - Controller1. Axis1. position();
    int LeftStick = Controller1. Axis3. position() + Controller1. Axis1. position();

    int IntakeStick = Controller2.Axis3. position();

    bool BackUp = Controller1.ButtonX.pressing();
    bool BackDown = Controller1.ButtonB.pressing();

    bool ArmUp = Controller1.ButtonY.pressing();

    bool Intake100 = Controller2.ButtonA.pressing();

    bool Intake1100 = Controller1.ButtonX.pressing();

    bool IntakeUp = Controller2.ButtonX.pressing();

    bool IntakeDown = Controller2.ButtonB.pressing();

    bool BackLift = Controller2.ButtonUp.pressing();
    bool BackLiftC = Controller1.ButtonUp.pressing();

    bool BackD = Controller2.ButtonDown.pressing(); 
    bool BackDC = Controller1.ButtonDown.pressing();

    bool CoverD = Controller2.ButtonR2.pressing();
    bool CoverU = Controller2.ButtonR1.pressing();

    leftEnableDrivePID = false;
    rightEnableDrivePID = false;

    //bool intakeSpin = Controller1.ButtonRight.pressing();

    //bool intakeStop = Controller1.ButtonLeft.pressing();

    //bool intakeCounter = Controller1.ButtonRight.pressing();

    /*
    if(intakeCounter == true){
      if(iS == true){
        iS = false;
      }
      else{
        if(iS == false){
          iS = true;
        }
      }
    }

    if(intakeSpin == true){
      if(iS == true){
        intake.spin(forward);
      }
      else{
        if(iS == false){
          intake.stop();
        }
      }
    }

    if(intakeStop == true){
      intake.spin(reverse);
    }
    */

    
    if(BackUp == true){
      BackUp = false;
      //Back.spinToPosition(-100, degrees, false);
    }

    if(BackDown == true){
      BackDown = false;
      //Back.spinToPosition(800, degrees, false);
    }

    Controller1.ButtonR2.pressed(PneumaticOut);

    Controller1.ButtonR1.pressed(PneumaticIn);

    if(ArmUp == true){
      Arm.spinToPosition(1200, degrees, false);
    }

    if(BackLift == true){
      BackLift1.set(true);
      BackLift2.set(true);
    }

    if(BackD == true){
      BackLift1.set(false);
      BackLift2.set(false);
    }

    if(BackLiftC == true){
      BackLift1.set(true);
      BackLift2.set(true);
    }

    if(BackDC == true){
      BackLift1.set(false);
      BackLift2.set(false);
    }

    if(CoverD == true){
      CoverD = false;
      Cover.set(true);
    }

    if(CoverU == true){
      CoverU = false;
      Cover.set(false);
    }

    if(abs(LeftStick)<deadband){
      LeftMotor.setVelocity(0, percent);
      ExtraLeft.setVelocity(0, percent);
    }
    else{
      LeftMotor.setVelocity(LeftStick, percent);
      ExtraLeft.setVelocity(LeftStick, percent);
    }

    if(abs(RightStick)<deadband){
      RightMotor.setVelocity(0, percent);
      ExtraRight.setVelocity(0, percent);
    }
    else{
      RightMotor.setVelocity(RightStick, percent);
      ExtraRight.setVelocity(RightStick, percent);
    }

    if(abs(IntakeStick)<deadband){
      intake.setVelocity(0, percent);
    }
    else{
      intake.setVelocity(IntakeStick, percent);
    }
    if(Intake100 == true){
      intake.setVelocity(100, percent);
    }

    if(Intake1100 == true){
      intake.setVelocity(100, percent);
    }

    if(IntakeUp == true){
      intake.setVelocity(intake.velocity(percent) + 1, percent);
    }
    if(IntakeDown == true){
      intake.setVelocity(intake.velocity(percent) + 1, percent);
    }

    RightMotor.spin(forward);
    LeftMotor.spin(forward);
    ExtraRight.spin(forward);
    ExtraLeft.spin(forward);
    intake.spin(reverse);




    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
