#include "input.h"

using namespace vex;

void tank_drive(double leftSpeedPercent,
                double rightSpeedPercent, 
                motor_group& left, 
                motor_group& right) {

    // Deadzones are the region where the input is so low it's registered as zero input.
    // Since controllers aren't precise, and don't return to "position 0" when you take 
    // your finger off, we need a deadzone value where we cut output when the input is too low.

    if (leftSpeedPercent > JOYSTICK_DEADZONE_PERCENT || leftSpeedPercent < -JOYSTICK_DEADZONE_PERCENT) {
        left.spin(directionType::fwd, leftSpeedPercent, percentUnits::pct);
    } else {
        left.stop(ROBOT_BRAKE_TYPE); //This is to make sure that when the input hits the deadzone, the bot stops moving.
    }
    if (rightSpeedPercent > JOYSTICK_DEADZONE_PERCENT || rightSpeedPercent < -JOYSTICK_DEADZONE_PERCENT) {
        right.spin(directionType::fwd, rightSpeedPercent, percentUnits::pct);
    } else {
        right.stop(ROBOT_BRAKE_TYPE); 
    }
}   


void arcade_drive(double horizontalChannel,
                  double verticalChannel,
                  motor_group& left,
                  motor_group& right) { 

    double straightSpeed = verticalChannel;
    double spinSpeed = horizontalChannel;

    // Dead zones are where the joystick value sets to zero within a certain
    // radius of the center.
    if (fabs(straightSpeed)< JOYSTICK_DEADZONE_PERCENT) {
        straightSpeed = 0;
    }
    if (fabs(spinSpeed)< JOYSTICK_DEADZONE_PERCENT) {
        spinSpeed = 0;
    }

    if (spinSpeed == 0 && straightSpeed == 0) {
        // This stops the robot if the user lets go of the joystick.
        left.stop(ROBOT_BRAKE_TYPE);
        right.stop(ROBOT_BRAKE_TYPE);
    } else {
        // This makes the robot move forward if the user moves the joystick.
        left.setVelocity(-(straightSpeed + spinSpeed), percent);
        right.setVelocity(-(straightSpeed - spinSpeed), percent);
        left.spin(forward);
        right.spin(forward);
    }
}