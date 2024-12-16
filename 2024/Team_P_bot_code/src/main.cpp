#include <algorithm>

#include "v5.h"
#include "v5_vcs.h"

/**
 * All VEX ports defined here for easy reassignment if needed.
 */
struct Port {
    // Drive motor ports
    static const int DRIVE_FRONT_LEFT = 4 - 1;
    static const int DRIVE_FRONT_RIGHT = 5 - 1;
    static const int DRIVE_BACK_LEFT = 6 - 1;
    static const int DRIVE_BACK_RIGHT = 7 - 1;

    // Intake motor port
    static const int INTAKE = 8 - 1;

    // Lift motor ports
    static const int LIFT_FRONT = 9 - 1;
    static const int LIFT_BACK = 10 - 1;
};

/**
 * Time (in msec) to wait between frames.
 */
const int WAIT_TIME = 20;

/**
 * Controls the drive subsystem by instantaneously adjusting the speed of the
 * drive motors.
 *
 * By having this as a separate function, we can call the same *tested* code in
 * both teleop and autonomous, and both will do the same thing to the drive.
 *
 * @param frontBackSpeed The speed in the front/back direction, from -100 (full
 *                       driving forward or backward is not desired.
 * @param turnSpeed      The angular velocity of the robot turning about its own
 *                       center, from -100 (full speed counterclockwise) to +100
 *                       (full speed clockwise.)  Use 0 if turning is not
 *                       desired.
 */
void drive(double frontBackSpeed, double turnSpeed) {

    // The drive motors (2 motors on each side of the drive base.)
    static vex::motor frontLeft(Port::DRIVE_FRONT_LEFT);
    static vex::motor frontRight(Port::DRIVE_FRONT_RIGHT);
    static vex::motor backLeft(Port::DRIVE_BACK_LEFT);
    static vex::motor backRight(Port::DRIVE_BACK_RIGHT);

    static vex::motor_group left(frontLeft, backLeft);
    static vex::motor_group right(frontRight, backRight);

    double leftMotorSpeed = frontBackSpeed + turnSpeed;
    double rightMotorSpeed = frontBackSpeed - turnSpeed;
    leftMotorSpeed = std::max(-100.0, std::min(leftMotorSpeed, 100.0));
    rightMotorSpeed = std::max(-100.0, std::min(rightMotorSpeed, 100.0));

    left.spin(vex::fwd, leftMotorSpeed, vex::velocityUnits::pct);
    right.spin(vex::fwd, rightMotorSpeed, vex::velocityUnits::pct);
}

/**
 * Controls the intake/uptake subsystem.  We can call this both during teleop
 * and autonomous.
 *
 * @param intakeOrOuttake Makes the pivot ramp intake or eject the rings.
 *                        Positive numbers trigger intake system, zero stops the
 *                        system, and negative numbers trigger outtake system.
 */
void intake(int intakeOrOuttake) {
    // The intake motor, used to collect rings.
    static vex::motor intake(Port::INTAKE);

    // When testing if Intake and Outake are reversed change the code for it to be
    // reversed.
    intake.setVelocity(100.00, vex::percentUnits::pct);
    if (intakeOrOuttake > 0) {
        // Intaking.
        intake.spin(vex::directionType::fwd);
    } else if (intakeOrOuttake < 0) {
        // Outtaking
        intake.spin(vex::directionType::rev);
    } else {
        // No movement
        intake.stop(vex::brakeType::brake);
    }
}

/**
 * Controls the lift subsystem. We can call this during teleop and autonomus.
 *
 * @param upOrDown Makes the Lift raise and lower pivot ramp. If the sign of the
 *             number is a positive, it triggers the lift to go up. Zero stops
 *             the system, and if the sign of the number is negative it triggers
 *             lowering system.
 */
void lift(int upOrDown) {
    static vex::motor frontLiftRight(Port::LIFT_FRONT);
    static vex::motor backLiftLeft(Port::LIFT_BACK);
    static vex::motor_group updownLift(frontLiftRight, backLiftLeft);

    updownLift.setVelocity(100.00, vex::percentUnits::pct);
    if (upOrDown > 0) {
        updownLift.spin(vex::directionType::fwd);
    } else if (upOrDown < 0) {
        updownLift.spin(vex::directionType::rev);
    } else {
        updownLift.stop(vex::brakeType::brake);
    }
}


/**
 * Callback function used to control the robot during the autonomous phase of a
 * VEX Competition.
 */
void autonomous() {
    // Insert autonomous user code here.
}

/**
 * Callback function used to control the robot during the user control phase of
 * a VEX Competition.
 */
void usercontrol() {
    vex::controller controller;
    // User control code here, inside the loop
    while (true) {
        // This is the main execution loop for the user control program.
        // Each time through the loop your program should update motor + servo
        // values based on feedback from the joysticks.

        double controllerFrontBackPosition = controller.Axis4.position();
        double controllerLeftRightPosition = controller.Axis3.position();
        drive(controllerFrontBackPosition, controllerLeftRightPosition);

        bool outtakePress = controller.ButtonL2.pressing();
        bool intakePress = controller.ButtonR2.pressing();
        int intakeOrOuttake = 0;
        if (intakePress) {
            intakeOrOuttake = 1;
        } else if (outtakePress) {
            intakeOrOuttake = -1;
        }

        intake(intakeOrOuttake);

        bool uplift = controller.ButtonUp.pressing();
        bool downlift = controller.ButtonDown.pressing();
        int liftUpOrDown = 0;
        if (uplift) {
            liftUpOrDown = 1;
        } else if (downlift) {
            liftUpOrDown = -1;
        }

        lift(liftUpOrDown);

        wait(WAIT_TIME, vex::msec);
    }
}

//
// Main will set up the competition and register callbacks.
//
int main() {
    vex::competition competition;

    // Set up callbacks for autonomous and driver control periods.
    competition.autonomous(autonomous);
    competition.drivercontrol(usercontrol);

    // Prevent main from exiting with an infinite loop.
    while (true) {
        wait(WAIT_TIME, vex::msec);
    }
}
