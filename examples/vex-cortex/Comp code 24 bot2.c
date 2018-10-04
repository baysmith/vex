#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ForwardRightEncoder, sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  RearRightEncoder, sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           mr,            tmotorVex393_HBridge, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port2,           fr,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           fl,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           am1,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           am2,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           co1,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           co2,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           feedbelt,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           barlift,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          ml,            tmotorVex393_HBridge, openLoop, encoderPort, I2C_1)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/* mr  is the Back  Right Motor
   fr  is the Front Right Motor
   ml  is the Back  Left Motor
   fl  is the Front Left Motor
   am1 is the       Arm Motor
   am2 is the       Arm motor
   co1 is the       Colletor Wheel Or Belt
   co2 is the       Colletor Wheel Or Belt
   feedbelt is
   barlift is
   */
#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(70)
#pragma userControlDuration(70)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!

/////////////////////////////////////////////////////////////////////////////////////////
//
//                          Pre-Autonomous Functions
//
// You may want to perform some actions before the competition starts. Do them in the
// following function.
//
/////////////////////////////////////////////////////////////////////////////////////////

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
	// Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}
void reset() //clear the motor encoders
{
	nMotorEncoder[mr] = 0;
	nMotorEncoder[ml] = 0;
}
void forward(int distance)
{
	slaveMotor(mr, fr); //Right Back motor does what ever the Right front motor does
	slaveMotor(ml, fl); //Left Back motor does what ever the Left front motor does
	int slow;
	int mid;
	slow = 60;
	mid = 90;
	reset();
	while(nMotorEncoder[ml] < distance)
	{
		if(nMotorEncoder[mr] == nMotorEncoder[ml]) // If rightEncoder has counted the same amount as leftEncoder:
		{
			// Move Forward
			motor[fr] = mid;
			motor[fl] = mid;

		}
		else if(nMotorEncoder[mr] > nMotorEncoder[ml])	// If rightEncoder has counted more encoder counts
		{
			// Turn slightly right
			motor[fr] = slow;
			motor[fl] = mid;
		}
		else	// Only runs if leftEncoder has counted more encoder counts
		{
			// Turn slightly left
			motor[fr] = mid;
			motor[fl] = slow;
		}
	}
}
void back(int distance)
{
		slaveMotor(mr, fr); //Right Back motor does what ever the Right front motor does
	slaveMotor(ml, fl); //Left Back motor does what ever the Left front motor does
	int slow;
	int mid;
	slow = -60;
	mid = -90;
	// Robot waits for 2000 milliseconds before executing program
	reset();
	while(nMotorEncoder[mr] > distance)
	{
		if(nMotorEncoder[mr] == nMotorEncoder[ml]) // If rightEncoder has counted the same amount as leftEncoder:
		{
			// Move Forward
			motor[fr] = mid;
			motor[fl] = mid;
		}
		else if(nMotorEncoder[mr] < nMotorEncoder[ml])	// If rightEncoder has counted more encoder counts
		{
			// Turn slightly right
			motor[fr] = slow;
			motor[fl] = mid;
		}
		else	// Only runs if leftEncoder has counted more encoder counts
		{
			// Turn slightly left
			motor[fr] = mid;
			motor[fl] = slow;
		}
	}
}
void turnl(int time)
{
	slaveMotor(mr, fr); //Right Back motor does what ever the Right front motor does
	slaveMotor(ml, fl); //Left Back motor does what ever the Left front motor does
	// Move forward at full power for 3 seconds
	motor[fr] = 127;	  // Motor on port3 is run at full (127) power forward
	motor[fl] = -127;		  // Motor on port5 is run at full (127) power forward
	wait1Msec(time);// Robot runs previous code for 3000 milliseconds before moving on
}
void turnr(int time)
{
	slaveMotor(mr, fr); //Right Back motor does what ever the Right front motor does
	slaveMotor(ml, fl); //Left Back motor does what ever the Left front motor does

	// Move forward at full power for 3 seconds
	motor[fr] = -127;		  // Motor on port3 is run at full (127) power forward
	motor[fl] = 127;		  // Motor on port5 is run at full (127) power forward
	wait1Msec(time);// Robot runs previous code for 3000 milliseconds before moving on
}
void clear()
{
	slaveMotor(mr, fr); //Right Back motor does what ever the Right front motor does
	slaveMotor(ml, fl); //Left Back motor does what ever the Left front motor does

	// Move forward at full power for 3 seconds
	motor[fr] = 0;			  // Motor on port3 is run at full (127) power forward
	motor[fl] = 0;		  // Motor on port5 is run at full (127) power forward
	// Robot runs previous code for 3000 milliseconds before moving on
}
/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task autonomous()
{


	// .....................................................................................
	// Insert user code here.
	// .....................................................................................

	AutonomousCodePlaceholderForTesting();  // Remove this function call once you have "real" code.
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 User Control Task
//
// This task is used to control your robot during the user control phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task usercontrol()
{
	// User control code here, inside the loop
	int threshold = 10;   // helps to eliminate 'noise' from a joystick that isn't perfectly at (0,0)
	// feel free to change this to match your needs.
	int Speed1 = 32;
	int Speed2 = 64;
	int Speed3 = 95;
	int Speed4 = 127;
	int joy_x;            // will hold the X value of the analog stick (choices below)
  int joy_y;            // will hold the Y value of the analog stick (choices below)

	slaveMotor(mr, fr); //Right Back motor does what ever the Right front motor does
	slaveMotor(ml, fl); //Left Back motor does what ever the Left front motor does
	slaveMotor(am2, am1); //Left Back motor does what ever the Left front motor does
	slaveMotor(co2, co1); //Left Back motor does what ever the Left front motor does
	slaveMotor(feedbelt, co1); //Left Back motor does what ever the Left front motor does

	while (true)
	{
		// This is the main execution loop for the user control program. Each time through the loop
		// your program should update motor + servo values based on feedback from the joysticks.

		// .....................................................................................
		// Insert user code here. This is where you use the joystick values to update your motors, etc.
		// .....................................................................................
		//Controller 1

    joy_x = vexRT[Ch4];   // This is the RIGHT analog stick.  For LEFT, change 'Ch1' to 'Ch4'.
    joy_y = vexRT[Ch3];   // This is the RIGHT analog stick.  For LEFT, change 'Ch2' to 'Ch3'.

    // Forward, and swing turns: (both abs(X) and abs(Y) are above the threshold, and Y is POSITIVE)
    if((abs(joy_x) > threshold) && (abs(joy_y) > threshold) && (joy_y > 0))
    {
      motor[fl]  = (joy_y + joy_x);
      motor[fr] = (joy_y - joy_x);
    }
    // Backwards and swing turns: (both abs(X) and abs(Y) are above the threshold, and Y is NEGATIVE)
    else if((abs(joy_x) > threshold) && (abs(joy_y) > threshold) && (joy_y < 0))
    {
      motor[fl]  = (joy_y - joy_x);
      motor[fr] = (joy_y + joy_x);
    }
    // Turning in place: (abs(X) is above the threshold, abs(Y) is below the threshold)
    else if((abs(joy_x) > threshold) && (abs(joy_y) < threshold))
    {
      motor[fl]  = joy_x;
      motor[fr] = (-1 * joy_x);
    }
    // Standing still: (both abs(X) and abs(Y) are below the threshold)
    else
    {
      motor[fl]  = 0;
      motor[fr] = 0;
    }


		if(abs(vexRT[Ch2]) > threshold)         // If the right joystick is greater than or less than the threshold:
		{
			motor[am1] = (vexRT[Ch2]);   // Right Joystick Y value / 2.
		}
		else                                    // If the right joystick is within the threshold:
		{
			motor[am1] = 0;                // Stop the right motor (cancel noise)
		}
		if(vexRT[Btn5U] == 1)
		{
			motor[co1] = Speed4;
		}
		else if(vexRT[Btn5D] == 1)
		{
			motor[co1] = -Speed4;
		}
		else
		{
			motor[co1] = 0;
		}


		//Controller 2
		if(abs(vexRT[Ch3Xmtr2]) > threshold)         // If the right joystick is greater than or less than the threshold:
		{
			motor[am1] = (vexRT [Ch3Xmtr2]);   // Right Joystick Y value / 2.
		}
		else                                    // If the right joystick is within the threshold:
		{
			motor[am1] = 0;                // Stop the right motor (cancel noise)
		}

		if(abs(vexRT[Ch2Xmtr2]) > threshold)         // If the right joystick is greater than or less than the threshold:
		{
			motor[barlift] = (vexRT [Ch2Xmtr2]);   // Right Joystick Y value / 2.
		}
		else                                    // If the right joystick is within the threshold:
		{
			motor[barlift] = 0;                // Stop the right motor (cancel noise)
		}

		if(vexRT[Btn5UXmtr2] == 1)
		{
			motor[co1] = Speed1;
		}
		else if(vexRT[Btn5DXmtr2] == 1)
		{
			motor[co1] = -Speed1;
		}
		else
		{
			motor[co1] = 0;
		}
	}
}
