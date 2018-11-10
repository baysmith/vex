#pragma config(Motor,  port2,           climb,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           frontRight,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           backRight,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           frontLeft,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           backLeft,      tmotorVex393_MC29, openLoop)
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*  Description: Double reverse four bar arm lift with a holonomic mechanum drive.     */
/*                                                                           */
/*---------------------------------------------------------------------------*/

/*+++++++++++++++++++++++++++++++++++++++++++++| Notes |++++++++++++++++++++++++++++++++++++++++++++++
Mecanum Drive with Deadzone Thresholds
- This program allows you to remotely control a robot with mecanum wheels.
- The left joystick Y-axis controls the robot's forward and backward movement.
- The left joystick X-axis controls the robot's left and right movement.
- The right joystick X-axis controls the robot's rotation.
- This program incorportes a threshold/deadzone that allows very low Joystick values to be ignored.
This allows the robot to ignore values from the Joysticks when they fail to center at 0,
and provides a margin of error for the driver when they only want the robot to move in one axis.
----------------------------------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

//GLOBAL VARIBLES: control acceleration of the robot.
float realLeftRight = 0.0;
float realForwardBack = 0.0;
float realTurn = 0.0;
const float acceleration = 0.1;

// MECHANUM DRIVE
// Given a holonomic, 4-wheel Mecanum drive and three joystick values, this
// function moves the robot in the desired direction.
//
// To strafe left and right, simply provide a value for leftRight (and maybe
// forwardBack) without providing a value for the turning channel.
//
// Remember that this function provides instantaneous motion, and must be
// called in a loop to take place continuously.
//
// Arguments:
// - leftRight: The value sent by the left-right joystick channel (-127 for
//   full leftward motion and 127 for full rightward motion.)
//
//   This is usualy Ch3 on the left joystick.
//
// - forwardBack: The value sent by the forward-back joystick channel (-127
//   for full leftward motion and 127 for full rightward motion.)
//
//   This is usually Ch4 on the left joystick.
//
// - turn: The value sent by the turning joystick channel (-127 to turn left
//   at full speed and 127 to turn right at full speed.)
//
//   This is usually Ch1 on the right joystick.

void mecanumDrive(int leftRight, int forwardBack, int turn) {

	// Don't let the controller drive the motors directly.  Instead, the controller
	// represents the desired state, and we increment our way towards that.
	realLeftRight += sgn(leftRight - realLeftRight) * acceleration;
	realForwardBack += sgn(forwardBack - realForwardBack) * acceleration;
	realTurn += sgn(turn - realTurn) * acceleration;

	if (leftRight < -127) {
		leftRight = -127;
	}
	if (leftRight > 127) {
		leftRight = 127;
	}
	if (forwardBack < -127) {
		forwardBack = -127;
	}
	if (forwardBack > 127) {
		forwardBack = 127;
	}

	if (turn < -127) {
		turn = -127;
	}
	if (turn > 127) {
		turn = 127;
	}

	motor[frontRight] = realForwardBack - realTurn - realLeftRight;
	motor[backRight] =  realForwardBack - realTurn + realLeftRight;
	motor[frontLeft] = realForwardBack + realTurn + realLeftRight;
	motor[backLeft] =  realForwardBack + realTurn - realLeftRight;

	// Determine when to activate the center climbing assistance wheels.
	motor[climb] = forwardBack;
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks
	// running between Autonomous and Driver controlled modes. You will need to
	// manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

// Uses the mecanumDrive() function to drive in set patterns, testing whether everything was wired correctly.
//
// Remember that this function provides instantaneous motion, and must be
// called in a loop to take place continuously.
//
// Arguments:
// - periodLengthMilliseconds: The duration of the programmed cycle of behavior.
//   Must be less than 32,767 (the maximum duration of the vex timers.)
//
// Return values:
// - True if we've reached the end of our cycle, false otherwise.  The cycle
//   will repeat itself again naturally unless you stop calling autonomousTest().
bool autonomousTest(int periodLengthMilliseconds) {

	float milliseconds = time1[timer1];
	if (milliseconds > periodLengthMilliseconds) {
		clearTimer(timer1);
		milliseconds = 0;
	}
	const float L = periodLengthMilliseconds;

	if (milliseconds >= 0.0 * L && milliseconds < 0.2 * L) {
		// Drive forward.
		mecanumDrive(0, 127, 0);
		} else if (milliseconds >= 0.2 * L && milliseconds < 0.4 * L) {
		// Drive right.
		mecanumDrive(127, 0, 0);
		} else if (milliseconds >= 0.4 * L && milliseconds < 0.6 * L) {
		// Drive backward.
		mecanumDrive(0, -127, 0);
		} else if (milliseconds >= 0.6 * L && milliseconds < 0.8 * L) {
		// Drive left.
		mecanumDrive(-127, 0, 0);
		} else {
		// Stop.
		mecanumDrive(0, 0, 0);
		return true;
	}
	return false;
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

task autonomous()
{
	// ..........................................................................
	// Insert user code here.
	// ..........................................................................

	// Remove this function call once you have "real" code.
	AutonomousCodePlaceholderForTesting();
}

// MECHANUM CONTROL
// Given a holonomic, 4-wheel Mecanum drive and three joystick channels, this
// function calls mecanumDrive() correctly to allow a human using those
// joysticks to drive the bot.
//
// Remember that this function provides instantaneous motion, and must be
// called in a loop to take place continuously.
//
// Arugments:
// - leftRightJoystickChannel: The channel that controls horizontal
//   strafing -- ordinarily Ch3 on the left joystick.
// - frontBackJoystickChannel: The channel that controls front-back
//   movement -- ordinarily Ch4 on the left joystick.
// - turnJoystickChannel: The channel that controls
//   counterclockwise/clockwise turning -- ordinarily Ch1 on the right joystick.
// - deadzoneThreshold: Any channel value whose aboslute value is less than this
//   will be ignored.  This compensates for the fact that joysticks sometimes
//   tend not to rest at perfect 0.
//   This variable will only have effect if it is non-negative.  Setting it too
//   high will make driving unresponsive and difficult.
void mecanumControl(int leftRightJoystickChannel, int frontBackJoystickChannel, int turnJoystickChannel, int deadzoneThreshold=15) {

	//Create "deadzone" variables. Adjust threshold value to increase/decrease deadzone
	int X2 = 0, Y1 = 0, X1 = 0;

	//Create "deadzone" for Y1/Ch3
	if(abs(vexRT[frontBackJoystickChannel]) > deadzoneThreshold)
		Y1 = vexRT[frontBackJoystickChannel];
	else
		Y1 = 0;

	//Create "deadzone" for X1/Ch4
	if(abs(vexRT[leftRightJoystickChannel]) > deadzoneThreshold)
		X1 = vexRT[leftRightJoystickChannel];
	else
		X1 = 0;

	//Create "deadzone" for X2/Ch1
	if(abs(vexRT[turnJoystickChannel]) > deadzoneThreshold)
		X2 = vexRT[turnJoystickChannel];
	else
		X2 = 0;

	mecanumDrive(X1, Y1, X2);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
	// User control code here, inside the loop

	int autonomousControl = 0;
	while (true)
	{
		// Button 5U (the upper left trigger) toggles "autonomomous"	vs. manual mode,
		// but it must be help down for at least half a second.
		if (autonomousControl == 0 && vexRT[Btn5U] > 0) {
			autonomousControl = 1;
			clearTimer(timer2);
			} else if (autonomousControl == 1) {
			if (vexRT[Btn5U] > 0) {
				if (time1[timer2] > 500) {
					// Button held down for 5 seconds--activate.
					autonomousControl = 2;
					clearTimer(timer2);
				}
				} else {
				// Button released before we could switch; autonomous canceled.
				autonomousControl = 0;
			}
			}	else if (autonomousControl == 2) {
			// If the button is pressed again, we cancel, but only after a threshold
			// (to allow the human to release the button.)
			if (vexRT[Btn5U] > 0 && time1[timer2] > 500) {
				autonomousControl = 0;
				} else {
				autonomousTest(10000);
			}
		}

		// If we're not driving in a square, the human can have a go.
		if (autonomousControl != 2) {
			//Remote Control Commands
			int threshold = 50;
			mecanumControl(Ch4, Ch3, Ch1, threshold);
		}

	} // end (while true)
} // end (task usercontrol)
