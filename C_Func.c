/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                          IHRC 421C Function Library                         */
/*                               VRC In The Zone                           	   */
/*															  	2017-2018																	 */
/*                                                                             */
/*-----------------------------------------------------------------------------*/




//CHASSIS---------------------------------------------------------------------------------------------------------------
int Y1 = 0, X2 = 0;						// Joystick drive and rotation values
float Y1_Mult	= 0.82;					// Multiplier to slow down in-place turning
float driveMultRF = 1;					  // Drive multipliers account for natural
float driveMultRB = 1;						// differences in motor RPMs
float driveMultLF = 1;						// (use Cody's RPM tester to find drive
float driveMultLB = 1;						// mult values
#define DEADZONE 	10					// Joystick deadzone
#define THRESHOLD 200					// threshold value


int rawLF = 0, rawLB = 0, rawRF = 0, rawRB = 0;

void trimDrive() // Function to trim motor values to an acceptable range.
{
	if(fabs(rawLF) > 127)					// Trims power level to usable bounds [-127, 127]
		rawLF = 127 * sgn(rawLF);
	if(fabs(rawLB) > 127)					// Trims power level to usable bounds [-127, 127]
		rawLB = 127 * sgn(rawLB);
	if(fabs(rawRF) > 127)					// Trims power level to usable bounds [-127, 127]
		rawRF = 127 * sgn(rawRF);
	if(fabs(rawRB) > 127)					// Trims power level to usable bounds [-127, 127]
		rawRB = 127 * sgn(rawRB);
}

void driveMove(int power) // Autonomous function to drive straight at a power level for a duration
{
	motor[driveLF] = motor[driveLB] = power;
	motor[driveRF] = motor[driveRB] = power;
}

void driveStop() // Autonomous function to stop for a duration
{
	motor[driveLF] = motor[driveLB] = 0;
	motor[driveRF] = motor[driveRB] = 0;
}

void driveTurnL (int power) // Autonomous function to turn left at a power level for a duration
{
	motor[driveLF] = motor[driveLB] = -power;
	motor[driveRF] = motor[driveRB] =  power;

}

void driveTurnR (int power) // Autonomous function to turn right at a power level for a duration
{
	motor[driveLF] = motor[driveLB] =  power;
	motor[driveRF] = motor[driveRB] = -power;
}

//LIFT------------------------------------------------------------------------------------------------------------------
#define LIFT_MAX 	1780			// Lift up position
#define LIFT_MIN 	410				// Lift down position

float liftKp = 0.52; 			   	// Proportional constant
float liftKi = 0.000004;   		// Integration constant
float liftKd = 0.07; 			    // Derivation constant
int liftCurrent;							// Current lift angle
int liftTarget;								// Target lift angle
int liftPower = 0;		    	 	// Value to be sent to lift motors

task liftPID()
{
	int liftError;									// Difference between Target and Current
	float liftProp;						      // Proportional term
	float liftInteg;						   	// Integral term
	float liftDeriv;								// Derivative term
	int		liftIntegThreshold = 2;	  // Range of error values in which the total error for the integral term accumulates
	float liftErrorTotal;						// Total error accumulated
	float liftErrorLast;				   	// Last error recorded by the controller

	while(true)
	{
		liftCurrent = SensorValue[liftPot];		// Read the sensor value
		liftError = liftTarget - liftCurrent; // Calculates difference between current angle and target angle

		if(liftError < liftIntegThreshold && liftErrorTotal != 0)	// Total error accumulates only when error exists and is within the threshold
			liftErrorTotal += liftError;														// Adds error to the total each time through the loop
		else
			liftErrorTotal = 0;								// if error = zero or error is not withing the active zone, total error is set to zero
		if(liftErrorTotal > (50 / liftKi))  // Caps total error at 50 to prevent integral windup
			liftErrorTotal = (50 / liftKi);
		if(liftError == 0)									// If error is zero derivative term is zero
			liftDeriv = 0;

		liftProp 	 =  liftError                  * liftKp; // Sets Proportion term
		liftInteg  =  liftErrorTotal             * liftKi; // Sets Integral term
		liftDeriv  = (liftError - liftErrorLast) * liftKd; // Sets Derivative term

		liftErrorLast = liftError; 	// Sets the last error to current error so we can use it in the next loop
		liftPower =  liftProp + liftInteg + liftDeriv; // Sets power level to total of all terms

		if(fabs(liftPower) > 127)						// Trims power level to usable bounds [-127, 127]
			liftPower = 127 * sgn(liftPower);

		// No threshhold to cut power so that PID process keeps arm in position
		motor[liftR] = motor[liftL] = liftPower; // Sends calculated value to motors
		wait1Msec(25); // Run the loop once every 25ms to prevent hogging the Cortex and causing instability
	}
}

//CLAW------------------------------------------------------------------------------------------------------------------
#define CLAW_OPEN 	2020		// Claw opened position
#define CLAW_CLOSE	1200		// Claw closed position
bool clawHold = false;	// Toggle for claw
int clawHoldPower = 0;	// Additional holding power when claw is closed
float claw_kp = 0.1; 	// Claw proportionality constant
int clawState;					// Toggle to open (1) or close (0)
int clawTarget;					// Target claw angle
int clawCurrent;				// Current claw angle
int clawPower = 0; 			// Claw power level
int clawError;			// Difference between Target and Current

task clawPID()
{
	int clawCurrent;		// Current claw angle

	while(true)
	{
		clawCurrent = SensorValue[clawPot];			// Read the sensor value
		clawError		= clawTarget - clawCurrent; // Calculates difference between current angle and target angle
		clawPower		= claw_kp * clawError;			// Calculate power level

		if(fabs(clawPower) > 127)						// Trims power level to usable bounds [-127, 127]
			clawPower = 127 * sgn(clawPower);

		if(clawHold && clawCurrent < 2130) // Adds claw clamping power when the claw is closed
			clawHoldPower = 0;
		else if(!clawHold)	// When the claw is open do not add clamping power
			clawHoldPower = 0;

		motor[claw] = clawPower + clawHoldPower; // Adds claw clamping power to calculated value and sends value to motor
		wait1Msec(25);	// Run the loop once every 25ms to prevent hogging the Cortex and causing instability
	}
}


//Flip------------------------------------------------------------------------------------------------------------------
#define FLIP_MIN 	2250			  // Flipper level when back over mobile goal
#define FLIP_MAX 	400		  		// Flipper level in front of bot
#define FLIP_BACK 2360			  // Flipper level at end of flipAndRelease function
float flipKp = 0.12; 			   	// Proportional constant
float flipKi = 0.000004;   		// Integration constant
float flipKd = 0.07; 			    // Derivation constant
int flipState;								// Toggle btween front (0) back(1)
int flipCurrent;							// Current flip angle
int flipTarget;								// Target flip angle
int flipPower = 0;		    	 	// Value to be sent to flipper motors

task flipPID()
{
	int flipError;									// Difference between Target and Current
	float flipProp;						      // Proportional term
	float flipInteg;						   	// Integral term
	float flipDeriv;								// Derivative term
	int		flipIntegThreshold = 2;	  // Range of error values in which the total error for the integral term accumulates
	float flipErrorTotal;						// Total error accumulated
	float flipErrorLast;				   	// Last error recorded by the controller

	while(true)
	{
		flipCurrent = SensorValue[flipPot];		// Read the sensor value
		flipError = flipTarget - flipCurrent; // Calculates difference between current angle and target angle

		if(flipError < flipIntegThreshold && flipErrorTotal != 0)	// Total error accumulates only when error exists and is within the threshold
			flipErrorTotal += flipError;														// Adds error to the total each time through the loop
		else
			flipErrorTotal = 0;								// if error = zero or error is not withing the active zone, total error is set to zero
		if(flipErrorTotal > (50 / flipKi))  // Caps total error at 50 to prevent integral windup
			flipErrorTotal = (50 / flipKi);
		if(flipError == 0)									// If error is zero derivative term is zero
			flipDeriv = 0;

		flipProp 	 =  flipError                  * flipKp; // Sets Proportion term
		flipInteg  =  flipErrorTotal             * flipKi; // Sets Integral term
		flipDeriv  = (flipError - flipErrorLast) * flipKd; // Sets Derivative term

		flipErrorLast = flipError; 	// Sets the last error to current error so we can use it in the next loop
		flipPower =  flipProp + flipInteg + flipDeriv; // Sets power level to total of all terms

		if(fabs(flipPower) > 127)						// Trims power level to usable bounds [-127, 127]
			flipPower = 127 * sgn(flipPower);

		// No threshhold to cut power so that PID process keeps arm in position
		motor[flip] = flipPower; // Sends calculated value to motors
		wait1Msec(25); // Run the loop once every 25ms to prevent hogging the Cortex and causing instability
	}
}


//Mobile Goal------------------------------------------------------------------------------------------------------------------
#define MOBILE_MIN 	1170			  // Mobile level when in
#define MOBILE_MAX 	3000				// Mobile level when extended
#define MOBILE_MID  1900				// Mobile level when goal is at mid height (used for 20 point zone)
int mobileState;								// Toggle between extended (0) and in (1)
int mobileMidState;							// Toggle between mid height (0) and in (1)
float mobileKp = 0.55; 			   	// Proportional constant
float mobileKi = 0.000004;   		// Integration constant
float mobileKd = 0.09; 			    // Derivation constant
int mobileCurrent;							// Current mobile angle
int mobileTarget;								// Target mobile angle
int mobilePower = 0;		    	 	// Value to be sent to mobile motors

task mobilePID()
{
	int mobileError;									// Difference between Target and Current
	float mobileProp;						      // Proportional term
	float mobileInteg;						   	// Integral term
	float mobileDeriv;								// Derivative term
	int		mobileIntegThreshold = 2;	  // Range of error values in which the total error for the integral term accumulates
	float mobileErrorTotal;						// Total error accumulated
	float mobileErrorLast;				   	// Last error recorded by the controller

	while(true)
	{
		mobileCurrent = SensorValue[mobilePot];		// Read the sensor value
		mobileError = mobileTarget - mobileCurrent; // Calculates difference between current angle and target angle

		if(mobileError < mobileIntegThreshold && mobileErrorTotal != 0)	// Total error accumulates only when error exists and is within the threshold
			mobileErrorTotal += mobileError;														// Adds error to the total each time through the loop
		else
			mobileErrorTotal = 0;								// if error = zero or error is not withing the active zone, total error is set to zero
		if(mobileErrorTotal > (50 / mobileKi))  // Caps total error at 50 to prevent integral windup
			mobileErrorTotal = (50 / mobileKi);
		if(mobileError == 0)									// If error is zero derivative term is zero
			mobileDeriv = 0;

		mobileProp 	 =  mobileError                  * mobileKp; // Sets Proportion term
		mobileInteg  =  mobileErrorTotal             * mobileKi; // Sets Integral term
		mobileDeriv  = (mobileError - mobileErrorLast) * mobileKd; // Sets Derivative term

		mobileErrorLast = mobileError; 	// Sets the last error to current error so we can use it in the next loop
		mobilePower =  mobileProp + mobileInteg + mobileDeriv; // Sets power level to total of all terms

		if(fabs(mobilePower) > 127)						// Trims power level to usable bounds [-127, 127]
			mobilePower = 127 * sgn(mobilePower);

		if(mobileTarget == MOBILE_MIN && abs(MOBILE_MIN - mobileCurrent) <= THRESHOLD)
			mobilePower = 0;

		if(mobileTarget == MOBILE_MAX && abs(MOBILE_MAX - mobileCurrent) <= THRESHOLD)
			mobilePower = 0;

		// No threshhold to cut power so that PID process keeps arm in position
		motor[mobile] = mobilePower; // Sends calculated value to motors
		wait1Msec(25); // Run the loop once every 25ms to prevent hogging the Cortex and causing instability
	}
}


//GYRO------------------------------------------------------------------------------------------------------------------

void gyrocalibrate()// Old Gyro calibration function for pre_auton
{
	SensorType[in7] = sensorNone; // Completely clear out any previous sensor readings by setting the port to "sensorNone"
	wait1Msec(500);
	SensorType[in7] = sensorGyro; // Reconfigure Analog Port 7 as a Gyro sensor and allow time for ROBOTC to calibrate it
	wait1Msec(1400);
}

void gyroleftturn(int degrees10)
{
	//Adjust SensorScale to correct the scaling for your gyro
	//SensorScale[in6] = 260;
	//Adjust SensorFullCount to set the "rollover" point. A value of 3600 sets the rollover point to +/-3600
	//SensorFullCount[in8] = 3600;

	int error = 5; //Specify the amount of acceptable error in the turn
	//While the absolute value of the gyro is less than the desired rotation - 100...
	while(abs(SensorValue[in7]) < abs(degrees10) - 100)
		driveTurnL(50);
	EndTimeSlice();
	//Brief brake to eliminate some drift
	driveTurnR(5);
	wait1Msec(100);

	//Second while loop to move the robot more slowly to its goal, also setting up a range
	//for the amount of acceptable error in the system
	while(abs(SensorValue[in7]) > degrees10 + error || abs(SensorValue[in7]) < degrees10 - error)
	{
		if(abs(SensorValue[in7]) > degrees10)
			driveTurnR(30);
		else
			driveTurnL(30);
	}
	//Stop
	driveStop();
	wait1Msec(250);
}

void gyrorightturn(int degrees10)
{
	//Adjust SensorScale to correct the scaling for your gyro
	//SensorScale[in6] = 360;
	//Adjust SensorFullCount to set the "rollover" point. A value of 3600 sets the rollover point to +/-3600
	//SensorFullCount[in8] = 3600;

	//Specify the number of degrees for the robot to turn (1 degree = 10, or 900 = 90 degrees)
	//int degrees10 = 900;
	//Specify the amount of acceptable error in the turn
	int error = 5;

	//While the absolute value of the gyro is less than the desired rotation - 100...
	while(abs(SensorValue[in7]) < abs(degrees10) - 100)
		driveTurnR(50);
	EndTimeSlice();
	//Brief brake to eliminate some drift
	driveTurnL(5);
	wait1Msec(100);

	//Second while loop to move the robot more slowly to its goal, also setting up a range
	//for the amount of acceptable error in the system
	while(abs(SensorValue[in7]) > degrees10 + error || abs(SensorValue[in7]) < degrees10 - error)
	{
		if(abs(SensorValue[in7]) > degrees10)
			driveTurnL(30);
		else
			driveTurnR(30);
	}
	//Stop
	driveStop();
	wait1Msec(250);
}

float gyro_kp = .85;	// Gyro proportionality constant (.85)
//int fakegyro = 0;		// For testing purposes only, simluates gyro values in debugger
float gyroTarget;			// Target gyro angle
float gyroCurrent;		// Current robot angle
float gyroError;			// Difference between Target and Current
int 	gyroPower = 0; 	// Gyro power level
int 	gyroPower_L = 0;
int	 	gyroPower_R = 0;

void gyroPIDTurn(float angle)
{
	const unsigned short Linearize[128] = 		// Remapping array to linearize motor control values
	{																					// Compensates for non-linearity of control value vs speed curve
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Making this array local clears up the debugger windows
		0, 21, 21, 21, 22, 22, 22, 23, 24, 24,  // Array is const unsigned short to prevent ROBOTC download errors
		25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
		28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
		33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
		37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
		41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
		46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
		52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
		61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
		71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
		80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
		88, 89, 89, 90, 90,127,127,127
	};

	gyroTarget = angle;	// Set Target angle
	do
	{
		gyroCurrent = theGyro.angle;	// Use JPearman's Gyro Library to filter and scale gyro readings
		//gyroCurrent = fakegyro;			// Use for testing on PC emulator

		// Calculates Error while compensating for 0-360 rollover, Error value should never be greater than 180 deg.
		if(fabs(gyroTarget - gyroCurrent) > 180) // If target - current  is > 180 deg., range must be shifted
		{
			if(gyroTarget < 180) // If the target is smaller than 180, add 360 deg. to compensate
				gyroError = gyroTarget + 360 - gyroCurrent;

			else if(gyroTarget >= 180) // If the target is greater than 180 deg., subtract 360 deg. to compensate
				gyroError = gyroTarget - 360 - gyroCurrent;
		}
		else
			gyroError = gyroTarget - gyroCurrent; // Otherwise subtract current angle from target

		gyroPower	= (gyro_kp * gyroError) + 1; // Calculate power level

		gyroPower_L = -(gyroPower) - (38 * sgn(gyroPower));//25
		//motor[driveLF] = motor[driveLB] = gyroPower_L;
		gyroPower_R =  (gyroPower) + (38 * sgn(gyroPower));//25
		//motor[driveRF] = motor[driveRB] = gyroPower_R;

		if(fabs(gyroPower_L) > 127)						// Trims power level to usable bounds [-127, 127]
			gyroPower_L = 127 * sgn(-gyroPower);
		if(fabs(gyroPower_R) > 127)						// Trims power level to usable bounds [-127, 127]
			gyroPower_R = 127 * sgn(gyroPower);

		// Scale each sum to compensate for motor variation and linearize
		motor[driveRF] = driveMultRF * Linearize[fabs(gyroPower_R)] * sgn(gyroPower_R);
		motor[driveRB] = driveMultRB * Linearize[fabs(gyroPower_R)] * sgn(gyroPower_R);
		motor[driveLF] = driveMultLF * Linearize[fabs(gyroPower_L)] * sgn(gyroPower_L);
		motor[driveLB] = driveMultLB * Linearize[fabs(gyroPower_L)] * sgn(gyroPower_L);

		wait1Msec(5);	// Run the loop once every 5ms to prevent instability
	}
	while ( (fabs(gyroError) > 1)); // Exit the loop when the error is within 1 degree

	driveStop(); // Cut power to the drive motors
	gyroPower = gyroPower_L = gyroPower_R = 0; // Clear out the the drive variables
}

//LCD-------------------------------------------------------------------------------------------------------------------
task LCDVoltage()
{
	bLCDBacklight = true;									// Turn on LCD Backlight
	string mainBattery;										// Battery that plugs into Cortex, powers claw and drive
	string powerexpanderBattery;					// Battery that plugs into Power Expander, powers lift
	string backupBattery;									// Backup 9V battery that maintains connection
	unsigned long timeCurrent	 = 0;
	unsigned long timePrevious = 0;
	while(true)														// An infinite loop to keep the program running until you terminate it
	{
		clearLCDLine(0);										// Clear line 1 (0) of the LCD
		clearLCDLine(1);										// Clear line 2 (1) of the LCD
		//Display the Primary Robot battery voltage
		displayLCDString(0, 0, "MAIN  PWREX BKUP");
		displayLCDString(1, 0, "");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/994.0, 'V'); 						//Build the value to be displayed
		displayNextLCDString(mainBattery);
		//Display the Power Expander battery voltage
		displayLCDString(1, 6, "");
		sprintf(powerexpanderBattery, "%1.2f%c", (float)SensorValue[in1]/283.2, 'V');	//Build the value to be displayed
		displayNextLCDString(powerexpanderBattery);
		//Display the Backup battery voltage
		displayLCDString(1, 12, "");
		sprintf(backupBattery, "%1.1f%c", BackupBatteryLevel/994.0, 'V');							//Build the value to be displayed
		displayNextLCDString(backupBattery);

		timeCurrent = nSysTime; // Set timeCurrent to System Time
		if((timeCurrent - timePrevious) > 250)
		{
			timePrevious = timeCurrent; // Save current time as timePrevious for next loop
			if(((float)nImmediateBatteryLevel/994.0 <= 7.6) || ((float)SensorValue[in1]/283.2 <= 7.6))
			{
				if(!bLCDBacklight)
					bLCDBacklight = true;
				else
					bLCDBacklight = false;
			}
			else
				bLCDBacklight = true;
		}
		//Short delay for the LCD refresh rate
		wait1Msec(125);
	}
}

task LCDBatteryFlash()
{
	bLCDBacklight = true;	// Turn on LCD Backlight
	string mainBattery;
	string powerexpanderBattery;
	string backupBattery;
	unsigned long timeCurrent	 = 0;
	unsigned long timePrevious = 0;
	while(true)	// An infinite loop to keep the program running until you terminate it
	{
		timeCurrent = nSysTime;
		if((timeCurrent - timePrevious) > 200)
		{
			timePrevious = timeCurrent;
			if(((float)nImmediateBatteryLevel/994.0 <= 8.1) || ((float)SensorValue[in1]/283.2 <= 8.1))
			{
				if(!bLCDBacklight)
					bLCDBacklight = true;
				else
					bLCDBacklight = false;
			}
			else
				bLCDBacklight = true;
		}
		//Short delay for the LCD refresh rate
		wait1Msec(100);
	}
}

//AUTONOMOUS------------------------------------------------------------------------------------------------------------

// Autonomous Routine Potentiometer Selector
// This code runs once to determine which autonomous routine to run
// The mode is changed by adjusting a potentiometer on the back of the robot

int AutonPrgm = 0;	// Variable for autonomous routine selector
int AutonSelection;
string CurrentSelectAuton;
int			LCDButtons = 0;
void getAutonRoutine()
{
	if (0 <= SensorValue[modePot] && SensorValue[modePot] < 683)	// Divides the potentiometer range into 6 sections to select
		AutonPrgm = 1;
	else if (683 <= SensorValue[modePot] && SensorValue[modePot] < 1365)
		AutonPrgm = 2;
	else if (1365 <= SensorValue[modePot] && SensorValue[modePot] < 2048)
		AutonPrgm = 3;
	else if (2048 <= SensorValue[modePot] && SensorValue[modePot] < 2730)
		AutonPrgm = 4;
	else if (2730 <= SensorValue[modePot] && SensorValue[modePot] < 3413)
		AutonPrgm = 5;
	else if (3413 <= SensorValue[modePot] && SensorValue[modePot] <= 4095)
		AutonPrgm = 6;
}

void getAutonName()
{
	switch(AutonPrgm)	{
	case 1:		CurrentSelectAuton = "ProgrammingSkills";		break;				// Mode 1:
	case 2:		CurrentSelectAuton = "StationaryR";		break; 				// Mode 2:
	case 3:		CurrentSelectAuton = "StationaryL";		break;			// Mode 3:
	case 4:		CurrentSelectAuton = "Mogo5Right";	break;		// Mode 4:
	case 5:		CurrentSelectAuton = "Mogo5Left";	break; 					// Mode 5:
	case 6:		CurrentSelectAuton = "DoNothing";		break; 	// Mode 6:
	default:	// nothing
	}
}

void waitForPress()
{
	while(nLCDButtons == 0){}
	wait1Msec(5);
}

void waitForRelease()
{
	while(nLCDButtons != 0){}
	wait1Msec(5);
}

void AutonSelector()
{
	const short leftButton 		= 1;
	const short centerButton 	= 2;
	const short rightButton		= 4;
	clearLCDLine(0);
	clearLCDLine(1);
	while(nLCDButtons != centerButton && bIfiRobotDisabled == 1)
	{
		getAutonRoutine();
		getAutonName();
		displayLCDCenteredString(0, CurrentSelectAuton);
		displayLCDChar(1, 0, 200);
		displayLCDChar(1, 15, 199);
		wait1Msec(25);
	}
	AutonSelection = AutonPrgm;

	for (int i = 0; i < 3; i++) // Repeat 3 times
	{
		bLCDBacklight = false;
		clearLCDLine(0);
		clearLCDLine(1);
		wait1Msec(100);
		bLCDBacklight = true;
		displayLCDCenteredString(0, CurrentSelectAuton);
		displayLCDCenteredString(1, "SELECTED");
		wait1Msec(100);
	}
	displayLCDCenteredString(0, CurrentSelectAuton);
	displayLCDCenteredString(1, "SELECTED");
	wait1Msec(500);
}
