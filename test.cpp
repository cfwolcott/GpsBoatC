#include <stdio.h>      /* printf */
#include <stdlib.h>
#include <string.h>
#include <math.h>       /* sin */

#include <wiringPi.h>
#include <wiringSerial.h>

#include "includes.h"
#include "HMC58X3.h"
#include "ADXL345.h"

float GetCompassHeading( float declination );

// Compass
HMC58X3 cCompass;

// Accelerometer
ADXL345 cAccel;

int main ()
{
	char id_str[3];
	unsigned int counter = 0;

	printf("sizeof float: %i\n", sizeof(float) );
	printf("sizeof int: %i\n", sizeof(int) );

	//-----------------------
	printf("WireingPi ... ");
	wiringPiSetup();
	printf("OK\n");

    // Init compass class
    cCompass.init(false); // Dont set mode yet, we'll do that later on. 

	memset( id_str, 0, sizeof(id_str) );
	cCompass.getID( id_str );
	printf("Compass ID: %s\n", id_str );
/*
    // Calibrate HMC using self test, not recommended to change the gain after calibration.
	printf("Calibrating ... ");
	{
		int i;
		for(i=1; i<2; i++)
		{
			printf("\nCaling with %i\n", i);
    		if(cCompass.calibrate(i, 32)) // Use gain 1=default, valid 0-7, 7 not recommended.
			{
				break;
			}
		}
	}
*/
    // Single mode conversion was used in calibration, now set continuous mode
    cCompass.setMode(0);

	cAccel.init( ADXL345_ADDR_ALT_LOW );

	while(1)
	{
		system("clear");
		printf("Counter: %i\n", counter++);
		printf("Heading: %.1f\n", GetCompassHeading( -13.0 ) );

		delay( 500 );
	}

	return 0;
}

//-----------------------------------------------------------------------------------
float GetCompassHeading( float declination )
{
    // Calculate heading when the magnetometer is level, then correctfor signs of axis. 
    int cx, cy, cz;
 	int ax, ay, az;
   
    cCompass.getValues( &cx, &cy, &cz );
	printf("Compass Raw: x=%i, y=%i, z=%i\n", cx, cy, cz);

	cAccel.readAccel( &ax, &ay, &az );
	printf("Accel Raw: x= %i, y= %i, z= %i\n", ax, ay, az);
    
    float heading = atan2( cy, cx );

    // If you have an EAST declination, use += declinationAngle, if you
    // have a WEST declination, use -= declinationAngle 

    heading -= radians(declination);

    // Correct for when signs are reversed. 
    if(heading < 0)
    {
		heading += TWO_PI;
    }

    // Check for wrap due to addition of declination. 
    if(heading > TWO_PI)
    {
    	heading -= TWO_PI;
    }

    // Convert radians to degrees for readability.
    return degrees( heading );
//    return (heading * 180/M_PI);
}
