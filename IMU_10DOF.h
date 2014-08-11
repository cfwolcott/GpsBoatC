// IMU_10DOF.h
//
// Groups all the sensors of the 10 Degree Of Freedom IMU board
//
// Sensors are as follows:
// HMC5883L - 3-axis magnetometer
// ADXL345 - 3-axis accelerometer
// ITG3200 - 3-axis gyro
// ?? - Barometric sensor

#include "includes.h"

#include "ADXL345.h"
#include "HMC5883L.h"

#ifndef IMU_10DOF_h
#define IMU_10DOF_h

typedef struct
{
	float x, y, z;
} tVECTOR;

class IMU
{
	public:
		IMU();		
		void Init();

		// Higher-level functions
		int GetHeading( float deviation );
		
		// Compass (magnetometer) functions
		tVECTOR GetCompassRaw( void );
		tVECTOR GetCompassScaled( void );
		void CompassCalibrate( void );

		// Accelerometer functions
		tVECTOR GetAccelRaw( void );

	private:
		// Sensor classes
		ADXL345 	cAccel;
		HMC5883L	cCompass;
};

#endif	// IMU_10DOF_h
