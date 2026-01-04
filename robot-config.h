using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor intake;
extern digital_out POut;
extern controller Controller2;
extern signature Vision1__RED_M;
extern signature Vision1__YEL_M;
extern signature Vision1__SIG_3;
extern signature Vision1__SIG_4;
extern signature Vision1__SIG_5;
extern signature Vision1__SIG_6;
extern signature Vision1__SIG_7;
extern vision Vision1;
extern motor Arm;
extern motor ExtraRight;
extern motor ExtraLeft;
extern digital_out BackLift1;
extern digital_out BackLift2;
extern motor_group RightMotor;
extern motor_group LeftMotor;
extern inertial Inertial1;
extern distance LeftSonic;
extern distance RightSonic;
extern digital_out Cover;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );