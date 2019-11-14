#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FrontRightWheel = motor(PORT1, ratio18_1, false);
motor FrontLeftWheel = motor(PORT2, ratio18_1, false);
motor BackRightWheel = motor(PORT3, ratio18_1, false);
motor BackLeftWheel = motor(PORT4, ratio18_1, false);
motor LeftIntake = motor(PORT5, ratio18_1, false);
motor RightIntake = motor(PORT6, ratio18_1, false);
motor IntakeLift = motor(PORT7, ratio6_1, false);
motor TrayPusher = motor(PORT8, ratio18_1, false);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variables used for controlling motors based on controller inputs
bool Controller1RightShoulderControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_callback_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    // check the ButtonR1/ButtonR2 status to control IntakeLift
    if (Controller1.ButtonR1.pressing()) {
      IntakeLift.spin(forward);
      Controller1RightShoulderControlMotorsStopped = false;
    } else if (Controller1.ButtonR2.pressing()) {
      IntakeLift.spin(reverse);
      Controller1RightShoulderControlMotorsStopped = false;
    } else if (!Controller1RightShoulderControlMotorsStopped) {
      IntakeLift.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1RightShoulderControlMotorsStopped = true;
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_callback_Controller1);
}