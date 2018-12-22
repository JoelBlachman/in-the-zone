/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     gyroLib.c                                                    */
/*    Author:     James Pearman, edited by Calvin O'Brien of 8304H             */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

// Stop recursive includes
#ifndef __GYROLIB__
#define __GYROLIB__

/*-----------------------------------------------------------------------------*/
/** @file    gyroLib2.h																												 */
/** @brief   VEX gyro wrapper functions for ROBOTC														 */
/*-----------------------------------------------------------------------------*/

// Structure to hold global info for the gyro
typedef struct _gyroData {
	tSensors    port;           ///< analog port the gyro is connected to
	bool        valid;          ///< indicates gyro is initialized
	float       angle;          ///< angle in range 0 to 360 deg
	float       abs_angle;      ///< absolute angle, both positive and negative
	int         drift_error;    ///< accumulated error due to drift
} gyroData;

// local storage for the gyro calculations
static  gyroData    theGyro;

#define GYRO_DRIFT_THRESHOLD    3

/*-----------------------------------------------------------------------------*/
/** @brief display current gyro angle on the LCD for debug pruposes            */
/*-----------------------------------------------------------------------------*/

void GyroDebug(int displayLine)
{
	string str;

	if(theGyro.valid)
	{
		// display current value
		sprintf(str,"Gyro %5.1f   ", theGyro.angle );
		displayLCDString(displayLine, 0, str);
	}
	else
		displayLCDString(displayLine, 0, "Init Gyro.." );
}

/*-----------------------------------------------------------------------------*/
/** @brief  Task that polls the Gyro and calculates the angle of rotation      */
/*-----------------------------------------------------------------------------*/

task GyroTask()
{
	int     gyro_value;
	int     lastDriftGyro = 0;
	float   angle;
	float   old_angle 	= 0.0;
	float   delta_angle = 0.0;
	long    nSysTimeOffset;

	SensorValue[dgtl4] = 1; // Reset Indicator LED
	theGyro.valid = false; // Gyro readings invalid
	theGyro.abs_angle = 0; // clear absolute
	theGyro.drift_error = 0; // clear drift error
	SensorType[theGyro.port] = sensorNone; // Cause the gyro to reinitialize
	wait1Msec(400); // Wait 1/2 sec
	SensorType[theGyro.port] = sensorGyro; // Gyro should be motionless here
	wait1Msec(1300); // Wait 1/2 sec
	nSysTimeOffset = nSysTime; // Save the current system timer
	while(true) // loop forever
	{
		gyro_value = SensorValue[theGyro.port]; // get current gyro value (deg * 10)

		// Filter drift when not moving
		// check this every 250mS
		if((nSysTime - nSysTimeOffset) > 250)
		{
			if(abs(gyro_value - lastDriftGyro) < GYRO_DRIFT_THRESHOLD)
				theGyro.drift_error += (lastDriftGyro - gyro_value);

			lastDriftGyro = gyro_value;
			nSysTimeOffset = nSysTime;
		}

		angle = (gyro_value + theGyro.drift_error) / 10.0; 	// Create float angle, remove offset

		if(angle < 0) // normalize into the range 0 - 360
			angle += 360;

		theGyro.angle = angle; // store in struct for others
		delta_angle = angle - old_angle; // calculate change from last time
		old_angle   = angle;

		if(delta_angle > 180) 	// fix rollover
			delta_angle -= 360;
		if(delta_angle < -180)
			delta_angle += 360;

		theGyro.abs_angle = theGyro.abs_angle + delta_angle; 		// store absolute angle
		theGyro.valid = true; // We can use the angle

		if(theGyro.valid)
			SensorValue[dgtl4] = 0;
		else if(!theGyro.valid)
			SensorValue[dgtl4] = 1;

		wait1Msec(20); 		// Delay
	}
}

/*-----------------------------------------------------------------------------*/
/** @brief     Initialize the Gyro                                             */
/** @param[in] port the analog port that the gyro is connected to              */
/*-----------------------------------------------------------------------------*/
void GyroInit(tSensors port = in6)
{
	theGyro.port  = port;
	theGyro.valid = false;
	theGyro.angle = 0.0;
	theGyro.abs_angle = 0.0;

	startTask(GyroTask);
}

/*-----------------------------------------------------------------------------*/
/** @brief Reinitialize the gyro task                                          */
/*-----------------------------------------------------------------------------*/
/** @details
*   Cause the gyro to be reinitialized by stopping and then restarting the
*   polling task
*/
void GyroReinit()
{
	stopTask(GyroTask);
	startTask(GyroTask);
}

/*-----------------------------------------------------------------------------*/
/** @brief    Get the current gyro angle in degrees                            */
/** @returns  gyro angle in the range 0 to 360 deg                             */
/*-----------------------------------------------------------------------------*/
float GyroAngleDegGet()
{
	return(theGyro.angle);
}

/*-----------------------------------------------------------------------------*/
/** @brief    Get the current gyro angle in radians                            */
/** @returns  gyro angle in the range 0 to 2PI radians                         */
/*-----------------------------------------------------------------------------*/
float GyroAngleRadGet()
{
	return(theGyro.angle / 180.0 * PI);
}

/*-----------------------------------------------------------------------------*/
/** @brief    Get the current absolute gyro angle                              */
/** @returns  the accumuated absolute gyro angle in degrees                    */
/*-----------------------------------------------------------------------------*/
float GyroAngleAbsGet()
{
	return(theGyro.abs_angle);
}

/*-----------------------------------------------------------------------------*/
/** @brief    Get the validity of the gyro                                     */
/** @returns  true if the gyro is initialized and returning valid data         */
/*-----------------------------------------------------------------------------*/
bool GyroValidGet()
{
	return(theGyro.valid);
}

/*-----------------------------------------------------------------------------*/
/** @brief    ROBOTC gyro warning elination                                    */
/*-----------------------------------------------------------------------------*/
/** @details
* The ROBOTC warnings about unused functions drive me crazy, so we call
* everything here, including this function, to remove them
* do not use or call this function from your code!
*/

void GyroWarningEliminate()
{
	GyroDebug(0);
	GyroReinit();
	GyroAngleDegGet();
	GyroAngleRadGet();
	GyroAngleAbsGet();
	GyroValidGet();
	GyroWarningEliminate();
}

#endif  //__GYROLIB__
