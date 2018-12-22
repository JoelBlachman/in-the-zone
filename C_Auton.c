/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                          IHRC 421C Autnomous Library                        */
/*                               VRC In The Zone                           	   */
/*															  	2017-2018																	 */
/*                                                                             */
/*-----------------------------------------------------------------------------*/


//ProgrammingSkills----------------------------------------------------------------------------
task ProgrammingSkills()
{
	startTask(LCDBatteryFlash);
	startTask(clawPID);
	startTask(flipPID);
	startTask(mobilePID);

	clawTarget = CLAW_CLOSE;				//close claw around cone
	flipTarget = FLIP_MIN;
	mobileTarget = MOBILE_MIN;
	wait1Msec(100);

	while(SensorValue[liftPot] < 700) {				//lift up
		motor[liftL] = motor[liftR] = 127;
	}

	if(SensorValue[liftPot] >= 700) {
		motor[liftL] = motor[liftR] = 0;
	}

	mobileTarget = MOBILE_MAX;				//extend MOGO
	wait1Msec(600);

	driveMove(80);				//drive forward
	wait1Msec(900);

	driveStop();
	wait1Msec(400);

	mobileTarget = MOBILE_MIN;			//pick up MOGO
	wait1Msec(800);

	while(SensorValue[liftPot] > 950) {				//lift down
		motor[liftL] = motor[liftR] = -127;
	}

	if(SensorValue[liftPot] <= 950) {
		motor[liftL] = motor[liftR] = 0;
	}

	clawTarget = CLAW_OPEN;					//open claw to release cone on mogo
	wait1Msec(600);

	while(SensorValue[liftPot] < 700) {				//lift up
		motor[liftL] = motor[liftR] = 127;
	}

	if(SensorValue[liftPot] >= 700) {
		motor[liftL] = motor[liftR] = 0;
	}

	driveMove(-80);			//drive backwards
	wait1Msec(450);

	driveStop();
	wait1Msec(400);

	driveTurnL(80);			//turn 180 degrees
	wait1Msec(1200);

	driveStop();
	wait1Msec(400);

	driveMove(80);				//drive forward
	wait1Msec(500);

	driveStop();
	wait1Msec(400);

	mobileTarget = MOBILE_MAX;				//extend MOGO
	wait1Msec(1200);

	driveMove(-80);			//drive backwards
	wait1Msec(450);

	driveStop();
	wait1Msec(400);

}

//StationaryR----------------------------------------------------------------------------
task StationaryR()
{
	startTask(LCDBatteryFlash);
	startTask(clawPID);
	startTask(flipPID);
	startTask(mobilePID);

	clawTarget = CLAW_CLOSE;				//close claw around cone
	flipTarget = FLIP_MIN;
	mobileTarget = MOBILE_MIN;
	wait1Msec(100);

	while(SensorValue[liftPot] < 900) {				//lift up
		motor[liftL] = motor[liftR] = 127;
	}

	if(SensorValue[liftPot] >= 900) {
		motor[liftL] = motor[liftR] = 0;
	}

	driveMove(80);						//drive forward
	wait1Msec(575);

	driveStop();
	wait1Msec(400);

	while(SensorValue[liftPot] > 870) {				//lift down
		motor[liftL] = motor[liftR] = -127;
	}

	if(SensorValue[liftPot] <= 870) {
		motor[liftL] = motor[liftR] = 0;
	}

	clawTarget = CLAW_OPEN;					//open claw to release cone on stationary goal
	wait1Msec(400);

	driveMove(-80);					//drive backwards away from stationary goal
	wait1Msec(275);

	driveStop();
	wait1Msec(600);

	driveTurnR(80);						//turn right
	wait1Msec(500);

	driveStop();
	wait1Msec(100);

	driveMove(80);					//drive foward
	wait1Msec(625);

	driveStop();
	wait1Msec(200);

	driveTurnL(72);				//turn left
	wait1Msec(550);

	driveStop();
	wait1Msec(50);


}

//StationaryL----------------------------------------------------------------------------
task StationaryL()
{
	startTask(LCDBatteryFlash);
	startTask(clawPID);
	startTask(flipPID);
	startTask(mobilePID);

	clawTarget = CLAW_CLOSE;				//close claw around cone
	flipTarget = FLIP_MIN;
	mobileTarget = MOBILE_MIN;
	wait1Msec(100);

	while(SensorValue[liftPot] < 900) {				//lift up
		motor[liftL] = motor[liftR] = 127;
	}

	if(SensorValue[liftPot] >= 900) {
		motor[liftL] = motor[liftR] = 0;
	}

	driveMove(80);						//drive forward
	wait1Msec(575);

	driveStop();
	wait1Msec(400);

	while(SensorValue[liftPot] > 870) {				//lift down
		motor[liftL] = motor[liftR] = -127;
	}

	if(SensorValue[liftPot] <= 870) {
		motor[liftL] = motor[liftR] = 0;
	}

	clawTarget = CLAW_OPEN;					//open claw to release cone on stationary goal
	wait1Msec(400);

	driveMove(-80);					//drive backwards away from stationary goal
	wait1Msec(275);

	driveStop();
	wait1Msec(600);

	driveTurnL(80);						//turn left
	wait1Msec(500);

	driveStop();
	wait1Msec(100);

	driveMove(80);					//drive foward
	wait1Msec(625);

	driveStop();
	wait1Msec(200);

	driveTurnR(72);				//turn right
	wait1Msec(550);

	driveStop();
	wait1Msec(50);


}

//Mogo5Right----------------------------------------------------------------------------
task Mogo5Right()
{
	startTask(LCDBatteryFlash); // Start battery voltage monitoring task
	startTask(clawPID); // Start claw PID task
	startTask(mobilePID);	// Start mogo PID task
	startTask(flipPID); // Start flipper PID task

	clawState = 0;
	wait1Msec(25);

	motor[liftL] = motor[liftR] = 127;
	wait1Msec(100);

	motor[liftL] = motor[liftR] = 0;
	wait1Msec(50);

}

//Mogo5Left----------------------------------------------------------------------------
task Mogo5Left()
{
	startTask(LCDBatteryFlash); // Start battery voltage monitoring task
	startTask(clawPID); // Start claw PID task
	startTask(mobilePID);	// Start mogo PID task
	startTask(flipPID); // Start flipper PID task

	clawState = 0;
	wait1Msec(25);

	motor[liftL] = motor[liftR] = 127;
	wait1Msec(100);

	motor[liftL] = motor[liftR] = 0;
	wait1Msec(50);

}

//DoNothing----------------------------------------------------------------------------
task DoNothing()
{
	startTask(LCDVoltage);
	wait1Msec(15000);
}
