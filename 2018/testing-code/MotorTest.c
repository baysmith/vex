<<<<<<< HEAD
#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           foo,           tmotorVex393_HBridge, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port2,           foo2,          tmotorVex393_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port3,           foo3,          tmotorVex393_MC29, openLoop, encoderPort, I2C_3)
=======
#pragma config(Motor,  port1,           BackLeft,      tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           FrontRight,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           FrontLeft,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           ElbowRight,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           ElbowLeft,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           ShoulderRight, tmotorVex269_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           ShoulderLeft,  tmotorVex269_MC29, openLoop)
#pragma config(Motor,  port10,          BackRight,     tmotorVex393_HBridge, openLoop, reversed)
>>>>>>> e7a39787ee3bca8a37ee438b220b18c4a352b997
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
<<<<<<< HEAD
	int speed = 63;
	int margin = 10;

	nMotorEncoder[port1] = 0;
	nMotorEncoder[port2] = 0;
	nMotorEncoder[port3] = 0;


	//motor[port1] = -127;
	//motor[port2] = -127;
	//motor[port3] = -127;
	while(true) {
		if(abs(nMotorEncoder[port3]) < abs(nMotorEncoder[port2])) {
			motor[port3] = speed + margin;
			motor[port2] = speed;
		}
		else if (abs(nMotorEncoder[port2]) < abs(nMotorEncoder[port3])) {
			motor[port3] = speed;
			motor[port2] = speed + margin;
		}
		else {
			motor[port3] = speed;
			motor[port2] = speed;
		}
	}
=======
	motor[ShoulderRight] = -127;
	sleep(1000);

>>>>>>> e7a39787ee3bca8a37ee438b220b18c4a352b997
}
