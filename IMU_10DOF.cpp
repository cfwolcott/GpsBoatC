// IMU_10DOF.cpp
//
// Groups all the sensors of the 10 Degree Of Freedom IMU board
//
// Sensors are as follows:
// HMC5883L - 3-axis magnetometer
// ADXL345 - 3-axis accelerometer
// ITG3200 - 3-axis gyro
// ?? - Barometric sensor

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>

#include "IMU_10DOF.h"

//---------------------------------------------------------------
// local defines

//---------------------------------------------------------------
// local data

// These are just some values for a particular unit; it is recommended that
// a calibration be done for your particular unit.
tVECTOR 	m_max;
tVECTOR 	m_min;

//---------------------------------------------------------------  
// local function prototypes

int get_heading(tVECTOR from, tVECTOR m, tVECTOR a);
void tVECTOR_cross(const tVECTOR *a,const tVECTOR *b, tVECTOR *out);
float tVECTOR_dot(const tVECTOR *a,const tVECTOR *b);
void tVECTOR_normalize(tVECTOR *a);

//------------------------------------------------------------------------------
IMU::IMU()
{
	// Min/Max values for magnotometer (as tested)
	m_max.x = 389.0; m_max.y = 583.0; m_max.z = 356.0;
	m_min.x = -322.0; m_min.y = -273.0; m_min.z = -450.0;
}

//------------------------------------------------------------------------------
void IMU::Init()
{
	// Accelerometer
	cAccel.Init( ADXL345_ADDR_ALT_LOW );
	cAccel.setRangeSetting( 2 );

	// Compass
	cCompass.Init();
	cCompass.SetScale(1.3); // Set the scale of the compass.
  	cCompass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
}

//------------------------------------------------------------------------------
int IMU::GetHeading( float deviation )
{
	int heading;

	// tVECTORs for compass and accelerometer
	tVECTOR m;
	tVECTOR a;
	
//	static int last_cx, last_cy, last_cz;
//	static int last_ax, last_ay, last_az;

	// Get compass values	
	m = GetCompassScaled();

//	cx = TOOLS_LowPassFilter( last_cx, cx );
//	cy = TOOLS_LowPassFilter( last_cy, cy );
//	cz = TOOLS_LowPassFilter( last_cz, cz );

	// Get accelerometer values
	a = GetAccelRaw();

//	ax = TOOLS_LowPassFilter( last_ax, ax );
//	ay = TOOLS_LowPassFilter( last_ay, ay );
//	az = TOOLS_LowPassFilter( last_az, az );

	heading = get_heading((tVECTOR){0,-1,0}, m, a);
 
    // If you have an EAST declination, use += declinationAngle, if you
    // have a WEST declination, use -= declinationAngle 

	heading += deviation;

    // Correct for when signs are reversed. 
    if( heading < 0 )
    {
		heading += TWO_PI;
    }

    // Check for wrap due to addition of declination. 
    if( heading > TWO_PI )
    {
    	heading -= TWO_PI;
    }

	return heading;
}

//------------------------------------------------------------------------------
tVECTOR IMU::GetCompassRaw( void )
{
	tVECTOR v;
	
	// Retrive the raw values from the compass (not scaled).
	MagnetometerRaw raw = cCompass.ReadRawAxis();

	v.x = raw.XAxis;
	v.y = raw.YAxis;
	v.z = raw.ZAxis;

	printf("Compass Raw: x= %i, y= %i, z= %i\n", raw.XAxis, raw.YAxis, raw.ZAxis);

	return v;
}

//------------------------------------------------------------------------------
tVECTOR IMU::GetCompassScaled( void )
{
	tVECTOR v;

	// Retrived the scaled values from the compass (scaled to the configured scale).
	MagnetometerScaled scaled = cCompass.ReadScaledAxis();

	printf("Compass Scaled: x= %f, y= %f, z= %f\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);

	v.x = scaled.XAxis;
	v.y = scaled.YAxis;
	v.z = scaled.ZAxis;

	return v;
}

//------------------------------------------------------------------------------
tVECTOR IMU::GetAccelRaw( void )
{
	tVECTOR v;
	float axyz[3];

	cAccel.get_Gxyz( axyz );

	printf("Accel Raw: x= %f, y= %f, z= %f\n", axyz[0], axyz[1], axyz[2] );

	v.x = axyz[0];
	v.y = axyz[1];
	v.z = axyz[2];

	return v;
}

//------------------------------------------------------------------------------
void IMU::CompassCalibrate( void )
{
#if 0
	if(cx > m_max.x) m_max.x = cx;
	if(cy > m_max.y) m_max.y = cy;
	if(cz > m_max.z) m_max.z = cz;
	
	if(cx < m_min.x) m_min.x = cx;
	if(cy < m_min.y) m_min.y = cy;
	if(cz < m_min.z) m_min.z = cz;

	printf("m_max: %f, %f, %f\n", m_max.x, m_max.y, m_max.z);
	printf("m_min: %f, %f, %f\n", m_min.x, m_min.y, m_min.z);
#endif
}

//------------------------------------------------------------------------------
// Returns the number of degrees from the From tVECTOR projected into
// the horizontal plane is away from north.
// 
// Description of heading algorithm: 
// Shift and scale the magnetic reading based on calibration data to
// to find the North tVECTOR. Use the acceleration readings to
// determine the Down tVECTOR. The cross product of North and Down
// tVECTORs is East. The tVECTORs East and North form a basis for the
// horizontal plane. The From tVECTOR is projected into the horizontal
// plane and the angle between the projected tVECTOR and north is
// returned.
int get_heading(tVECTOR from, tVECTOR m, tVECTOR a)
{
    // shift and scale
    m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
    m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
    m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

    tVECTOR temp_a = a;
    // normalize
    tVECTOR_normalize(&temp_a);
    //tVECTOR_normalize(&m);

    // compute E and N
    tVECTOR E;
    tVECTOR N;
    tVECTOR_cross(&m, &temp_a, &E);
    tVECTOR_normalize(&E);
    tVECTOR_cross(&temp_a, &E, &N);
	
    // compute heading
    int heading = round(atan2(tVECTOR_dot(&E, &from), tVECTOR_dot(&N, &from)) * 180 / M_PI);

    if (heading < 0)
	{
		heading += 360;
	}

	return heading;
}

//------------------------------------------------------------------------------
void tVECTOR_cross(const tVECTOR *a,const tVECTOR *b, tVECTOR *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

//------------------------------------------------------------------------------
float tVECTOR_dot(const tVECTOR *a,const tVECTOR *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

//------------------------------------------------------------------------------
void tVECTOR_normalize(tVECTOR *a)
{
  float mag = sqrt(tVECTOR_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}


