// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftFront            motor         11              
// rightFront           motor         13              
// leftBack             motor         9               
// rightBack            motor         1               
// VisionSensor         vision        16              
// LeftLineTracker      line          B               
// RightLineTracker     line          C               
// Camera_motor         servo         A               
// LED_RED              led           D               
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\eshsp                                            */
/*    Created:      Fri Mar 24 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftFront            motor         11              
// rightFront           motor         13              
// leftBack             motor         9               
// rightBack            motor         1               
// VisionSensor         vision        16              
// LeftLineTracker      line          B               
// RightLineTracker     line          C               
// Camera_motor         servo         A               
// LED_RED              led           D               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "Off_Season_Team-1.h"
#include "vex.h"

using namespace vex;

signature hatedCube = VisionSensor__GREEN_CUBE;

// Drive is a way to call mechanum so both teliop and autonomus can use it.
// fowardBack is the amount of movement in the foward and back channel a
// positive
// value drives foward a negative drives backwards and 0 stops it.
// rotate is the amount of movement in the turning right and left channel, as
// with
// fowardBack; positive value drives foward a negative drives backwards and 0
// stops it.
// strafe is the amount of movement in the left and rivht channel a positive
// value drives foward a negative drives backwards and 0 stops it.

/*void drive(double fowardBack, double rotate, double strafe) {
  leftFront.setVelocity(fowardBack + rotate + strafe, percent);
  rightFront.setVelocity(fowardBack - rotate - strafe, percent);
  leftBack.setVelocity(fowardBack + rotate - strafe, percent);
  rightBack.setVelocity(fowardBack - rotate + strafe, percent);
}

void colorHater(int fowardBack, int rotate, int strafe) {
  VisionSensor.takeSnapshot(hatedCube);
  if (VisionSensor.largestObject.exists) {
    double x = VisionSensor.largestObject.centerX;
    double delta = -x;
    delta = delta < -1 ? -1 : delta > 1 ? 1 : delta;
    drive(0, delta, 0);
    LED_RED.on();
  }else {
    LED_RED.off();
  }
}*/

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  int fowardBack;
  int rotate;
  int strafe;
  int x = 0;
  int Xcenter = 150;
  int OKErrorX = 60;
  int rotateSpeed = 0;
  int y = 0;
  int Ycenter = 110;
  int OKErrorY = 40;
  int upDownSpeed = 0;
  int servoChange = 0;
    leftFront.setVelocity(100,percent);
          leftBack.setVelocity(100,percent);
          rightFront.setVelocity(100,percent);
          rightBack.setVelocity(100,percent);
  Camera_motor.setPosition(10,degrees);

  while (1 == 1) {

    VisionSensor.takeSnapshot(VisionSensor__GREEN_CUBE);
    if(VisionSensor.largestObject.exists){
      x = (VisionSensor.largestObject.centerX);
      rotateSpeed = (Xcenter-x)*0.5;

        if(x<(Xcenter-OKErrorX)){
          leftFront.spinFor(reverse, rotateSpeed, degrees, false);
          leftBack.spinFor(reverse, rotateSpeed, degrees, false);
          rightFront.spinFor(forward, rotateSpeed, degrees, false);
          rightBack.spinFor(forward, rotateSpeed, degrees);
        }
    VisionSensor.takeSnapshot(VisionSensor__GREEN_CUBE);
    x = (VisionSensor.largestObject.centerX);
    rotateSpeed = (Xcenter-x)*0.5;
        if(x>(Xcenter+OKErrorX)){
          leftFront.spinFor(reverse, rotateSpeed, degrees, false);
          leftBack.spinFor(reverse, rotateSpeed, degrees, false);
          rightFront.spinFor(forward, rotateSpeed, degrees, false);
          rightBack.spinFor(forward, rotateSpeed, degrees);
        }

    }
  else{
      leftFront.spinFor(forward, 30, degrees, false);
          leftBack.spinFor(forward, 30, degrees, false);
          rightFront.spinFor(reverse, 30, degrees, false);
          rightBack.spinFor(reverse, 30, degrees);
          Camera_motor.setPosition(10,degrees);
  }

        /*y = (VisionSensor.largestObject.centerY);
      upDownSpeed = (Ycenter-y)*0.6;

        if(y<(Ycenter-OKErrorY)){
          Camera_motor.setPosition(x,degrees);
        }
    VisionSensor.takeSnapshot(VisionSensor__GREEN_CUBE);
    x = (VisionSensor.largestObject.centerX);
    rotateSpeed = (Xcenter-x)*0.6;
        if(x>(Xcenter+OKErrorX)){
          leftFront.spinFor(reverse, rotateSpeed, degrees, false);
          leftBack.spinFor(reverse, rotateSpeed, degrees, false);
          rightFront.spinFor(forward, rotateSpeed, degrees, false);
          rightBack.spinFor(forward, rotateSpeed, degrees);
        }*/

    


  }

  
}