// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         6               
// IntakeLift           motor         7               
// TrayPusher           motor         8               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         6               
// IntakeLift           motor         7               
// TrayPusher           motor         8               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         6               
// IntakeLift           motor         7               
// TrayPusher           motor         8               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         6               
// IntakeLift           motor         7               
// TrayPusher           motor         8               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         6               
// IntakeLift           motor         7               
// TrayPusher           motor         8               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         6               
// IntakeLift           motor         7               
// TrayPusher           motor         8               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         6               
// IntakeLift           motor         7               
// TrayPusher           motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         6               
// IntakeLift           motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         6               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// LeftIntake           motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// BackLeftWheel        motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// BackRightWheel       motor         3               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// FrontLeftWheel       motor         2               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontRightWheel      motor         1               
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\jwilsoniii_23                                    */
/*    Created:      Wed Nov 13 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

void mecanumDrive (int leftRight, int forwardBack, int turn) {
  FrontRightWheel.spin(forward, forwardBack - turn - leftRight, percent);
  BackRightWheel.spin(forward, forwardBack - turn + leftRight, percent);
  FrontLeftWheel.spin(forward, forwardBack + turn + leftRight, percent);
  BackLeftWheel.spin(forward, forwardBack + turn - leftRight, percent);
}

int main() {
  IntakeLift.setVelocity(95, percent);
  
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  while(true) {
    // allows controller to control both intakes with one button (the left trigger)
    //left intake is automatically configured with robot config
    int v = LeftIntake.velocity(percent);
    RightIntake.setVelocity(v, percent);

    
    //axis 4 is left joysticks horizontal movement and will be used for straffing 
    int leftRight = Controller1.Axis4.position(percent);

    //axis 3 is the left joysticks vertical movement will be used for forward-backward movement
    int forwardBack = Controller1.Axis3.position(percent);

    //axis 1 is the right joystick horizontal movement and will be used for turning
    int turn = Controller1.Axis1.position(percent);

    //update drive motor values continously as the driver changes the joystick
    mecanumDrive(leftRight, forwardBack, turn);

    //This is a throw-away code for testing
    int upDown = Controller1.Axis2.position(percent);
    TrayPusher.spin(forward, upDown, percent);
  }
}
