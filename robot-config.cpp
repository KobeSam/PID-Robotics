#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor intake = motor(PORT15, ratio6_1, false);
digital_out POut = digital_out(Brain.ThreeWirePort.A);
controller Controller2 = controller(partner);
/*vex-vision-config:begin*/
signature Vision1__RED_M = signature (1, 10053, 11157, 10605, -1815, -1291, -1553, 2.5, 0);
signature Vision1__YEL_M = signature (2, 2593, 3329, 2961, -5519, -4865, -5192, 2.7, 0);
signature Vision1__SIG_3 = signature (3, -51, 51, 0, -51, 51, 0, 3, 0);
vision Vision1 = vision (PORT21, 50, Vision1__RED_M, Vision1__YEL_M, Vision1__SIG_3);
/*vex-vision-config:end*/
motor Arm = motor(PORT7, ratio36_1, true);
motor ExtraRight = motor(PORT5, ratio18_1, true);
motor ExtraLeft = motor(PORT3, ratio18_1, false);
digital_out BackLift1 = digital_out(Brain.ThreeWirePort.B);
digital_out BackLift2 = digital_out(Brain.ThreeWirePort.C);
motor RightMotorMotorA = motor(PORT8, ratio18_1, false);
motor RightMotorMotorB = motor(PORT4, ratio18_1, false);
motor_group RightMotor = motor_group(RightMotorMotorA, RightMotorMotorB);
motor LeftMotorMotorA = motor(PORT13, ratio18_1, true);
motor LeftMotorMotorB = motor(PORT6, ratio18_1, true);
motor_group LeftMotor = motor_group(LeftMotorMotorA, LeftMotorMotorB);
inertial Inertial1 = inertial(PORT14);
distance LeftSonic = distance(PORT16);
distance RightSonic = distance(PORT2);
digital_out Cover = digital_out(Brain.ThreeWirePort.D);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // check the ButtonL1/ButtonL2 status to control Arm
      if (Controller1.ButtonL1.pressing()) {
        Arm.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        Arm.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        Arm.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

// define variables used for controlling motors based on controller inputs
bool Controller2LeftShoulderControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller2
int rc_auto_loop_function_Controller2() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // check the ButtonL1/ButtonL2 status to control Arm
      if (Controller2.ButtonL1.pressing()) {
        Arm.spin(forward);
        Controller2LeftShoulderControlMotorsStopped = false;
      } else if (Controller2.ButtonL2.pressing()) {
        Arm.spin(reverse);
        Controller2LeftShoulderControlMotorsStopped = false;
      } else if (!Controller2LeftShoulderControlMotorsStopped) {
        Arm.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller2LeftShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
  task rc_auto_loop_task_Controller2(rc_auto_loop_function_Controller2);
}