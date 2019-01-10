#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
<<<<<<< HEAD
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           backLeft,      tmotorVex393_HBridge, openLoop, encoderPort, I2C_3)
#pragma config(Motor,  port2,           frontRight,    tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port8,           backRight,     tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port9,           frontLeft,     tmotorVex393_MC29, openLoop, encoderPort, I2C_4)
#pragma config(Motor,  port10,          center,        tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

=======
#pragma config(Motor,  port1,           backLeft,      tmotorVex393_HBridge, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port2,           frontRight,    tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port8,           backRight,     tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port9,           frontLeft,     tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port10,          center,        tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


>>>>>>> e7a39787ee3bca8a37ee438b220b18c4a352b997
// This code is for the mentor-bot.
//
// Hit button 8D and the robot will drive forward three feet...
// ...as measured from the floor of the El Camino Student Activity Center.
// ...as measured with the weight of the mentor-bot at the time.
// ...as measured with the mentor-bot's present battery levels.
// ...as measured with the mentor-bot's current wheels.
//
// Hit button 8L to turn 90 degrees...kinda!

float gyroVal = 0;
task main()
{

	const float ENCODER_CLICKS_PER_INCH = 42.432;//1000/24.5;
	int leftSpeed = 0;
	int rightSpeed = 0;
	bool drivingForward = false;
	int goalAngle = 0;
	bool turning = false;
	//Gyro is set to 0
	SensorType[in8] = sensorNone;
	wait1Msec(1000);
<<<<<<< HEAD
	//Reconfigure gyro and calibrates it
	SensorType[in8] = sensorGyro;
	wait1Msec(2000);
=======
 	//Reconfigure gyro and calibrates it
 	SensorType[in8] = sensorGyro;
 	wait1Msec(2000);
>>>>>>> e7a39787ee3bca8a37ee438b220b18c4a352b997


	while(true)
	{
		// Once we press the 8D button, we get a temporary burst of power to the drive.
		if (vexRT[Btn8D] > 0) {
			drivingForward = true;
			leftSpeed = 80;
			rightSpeed = 80;
			// We do not know if the encoder is set to zero, this will ensure that the values are.
			nMotorEncoder[backLeft] = 0;
			nMotorEncoder[backRight] = 0;
			nMotorEncoder[frontRight] = 0;
			nMotorEncoder[frontLeft] = 0;
		}

		// Only travel 3 feet.
		const float inches_per_foot = 12;
		const float feet = 3;
		if (drivingForward == true &&
<<<<<<< HEAD
			nMotorEncoder[frontRight] > feet * inches_per_foot * ENCODER_CLICKS_PER_INCH) {
=======
			  nMotorEncoder[frontRight] > feet * inches_per_foot * ENCODER_CLICKS_PER_INCH) {
>>>>>>> e7a39787ee3bca8a37ee438b220b18c4a352b997
			leftSpeed = 0;
			rightSpeed = 0;
			drivingForward = false;
		}

		if (vexRT[Btn8L] > 0) {
			float currentBearing = SensorValue[in8] / 10.0;
			goalAngle = currentBearing + 90;
			turning = true;
		}

		if (turning == true)
		{
			float currentBearing = SensorValue[in8] / 10.0;

			// What we SHOULDN'T do:
			// if (currentBearing > goalAngle) { ... }

			const float TOLERANCE = 2;
			if (goalAngle - currentBearing > TOLERANCE) {
				// Turn right.
				leftSpeed = 40;
				rightSpeed = -40;
<<<<<<< HEAD
				} else if (goalAngle - currentBearing < -TOLERANCE) {
				// Turn left.
				leftSpeed = -40;
				rightSpeed = 40;
				} else {
=======
			} else if (goalAngle - currentBearing < -TOLERANCE) {
				// Turn left.
				leftSpeed = -40;
				rightSpeed = 40;
			} else {
>>>>>>> e7a39787ee3bca8a37ee438b220b18c4a352b997
				// Reached the goal; cut speed.
				leftSpeed = 0;
				rightSpeed = 0;
				turning = false;
			}

		}

		motor[backLeft] = leftSpeed;
		motor[backRight] = rightSpeed;
		motor[frontRight] = rightSpeed;
		motor[frontLeft] = leftSpeed;

		// For debugging purposes, copy the gyro sensor to a global variable.
		gyroVal = SensorValue[in8]/10;
	}

}
