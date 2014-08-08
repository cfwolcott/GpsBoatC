// **************************************************************
// GPS Boat
// main.c
// **************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include "includes.h"
#include "config.h" // defines I/O pins, operational parameters, etc.
#include "tools.h"
#include "TinyGPS.h"
//#include "HMC58X3.h"
#include "HMC5883L.h"
#include "ADXL345.h"
#include "Arduino.h"

//---------------------------------------------------------------
// local defines

#define LED_PIN		2
#define LED_ON		digitalWrite (LED_PIN, HIGH) ;	// On
#define LED_OFF		digitalWrite (LED_PIN, LOW) ;	// Off

typedef enum
{
  E_GO_LEFT,
  E_GO_RIGHT,
  E_GO_STRAIGHT
} E_DIRECTION;

// Program State Machine states
typedef enum
{
    E_NAV_INIT,
    E_NAV_WAIT_FOR_GPS_LOCK,    // progresses to E_NAV_SET_NEXT_WAYPOINT
    E_NAV_WAIT_FOR_GPS_STABLIZE,
    E_NAV_WAIT_FOR_GPS_RELOCK,  // resumes navigation state to E_NAV_START if GPS loses lock
    E_NAV_SET_NEXT_WAYPOINT,
    E_NAV_START,
    E_NAV_RUN,
    E_NAV_STOP,
    E_NAV_IDLE,
    
    E_NAV_MAX
} E_NAV_STATE;

typedef struct
{
    float flat;
    float flon;
    float fmph;
    float fcourse;
    U8 hour;
    U8 minute;
    U8 second;
	bool bGpsLocked;
} tGPS_INFO;

typedef struct
{
	float dist_to_waypoint;
	float bear_to_waypoint;
	float current_heading;
} tNAV_INFO;

typedef struct
{
    float flat;
    float flon;
} tWAY_POINT;

typedef struct vector
{
	float x, y, z;
} vector;

//---------------------------------------------------------------
// local data

// Vectors for compass and accelerometer
vector m;
vector a;

// These are just some values for a particular unit; it is recommended that
// a calibration be done for your particular unit.
vector m_max;
vector m_min;

// Global program state
E_NAV_STATE geNavState;

// GPS
int gSerial_fd;
TinyGPS cGps;
tGPS_INFO gtGpsInfo;

// Compass
//HMC58X3 cCompass;
HMC5883L cCompass;

// Accelerometer
ADXL345 cAccel;

// Arduino on I2C bus
Arduino cArduino;

// Navigation Info
tNAV_INFO gtNavInfo;

int gTargetWP = 0;

// Way point table
tWAY_POINT gtWayPoint[] = {
#if USE_HOME_POSITION
    { WAYPOINT_HOME_LAT, WAYPOINT_HOME_LON },
#else
    { 0, 0 },                              // Home Waypoint (will set with inital lock)
#endif
    { WAYPOINT_A_LAT, WAYPOINT_A_LON },    // Waypoint A
    { WAYPOINT_B_LAT, WAYPOINT_B_LON },    // Waypoint B
    { WAYPOINT_C_LAT, WAYPOINT_C_LON },    // Waypoint C
    { WAYPOINT_D_LAT, WAYPOINT_D_LON },    // Waypoint D
    { WAYPOINT_E_LAT, WAYPOINT_E_LON },    // Waypoint E
    { WAYPOINT_F_LAT, WAYPOINT_F_LON },    // Waypoint F
    { WAYPOINT_G_LAT, WAYPOINT_G_LON },    // Waypoint G
    { WAYPOINT_H_LAT, WAYPOINT_H_LON },    // Waypoint H
    { WAYPOINT_I_LAT, WAYPOINT_I_LON },    // Waypoint I
    { WAYPOINT_J_LAT, WAYPOINT_J_LON },    // Waypoint J    
};


//---------------------------------------------------------------  
// local function prototypes

bool 	UpdateGps( tGPS_INFO *ptGpsInfo );
void    PrintProgramState( E_NAV_STATE eState );
E_DIRECTION DirectionToBearing( float DestinationBearing, float CurrentBearing, float BearingTolerance );
void    SetSpeed( int new_speed );
void    SetRudder( int new_setting );
float   GetCompassHeading( float declination );
void	setup( void );
void	loop( void );

int get_heading(vector from);
void vector_cross(const vector *a,const vector *b, vector *out);
float vector_dot(const vector *a,const vector *b);
void vector_normalize(vector *a);

//---------------------------------------------------------------
// main
//---------------------------------------------------------------
int main(int argc, char **argv)
{
	// Min/Max values for magnotometer (as tested)
	m_max.x = 389.0; m_max.y = 583.0; m_max.z = 356.0;
	m_min.x = -322.0; m_min.y = -273.0; m_min.z = -450.0;

	system("clear");
	printf("GpsBoat - Version 1.0\n\n");

	//-----------------------
	// Setup hardware
	//-----------------------
	printf("Setting up hardware:\n");
	setup();

	delay(3000);
 
	//-----------------------
	// Main Loop
	//-----------------------
	printf("Starting Main Loop:\n");
	while(1)
	{
		loop();

		// Print system status
//		system("clear");
		printf("Status: "); PrintProgramState( geNavState ); printf("\n");
		printf("Navigation Info:\n");
		printf("Bearing to Target: %i\n", gtNavInfo.bear_to_waypoint);
		printf("Distance to Target: %f meters\n", gtNavInfo.dist_to_waypoint);
		printf("Heading: %i\n", (U16)gtNavInfo.current_heading);
		printf("\n");
		printf("GPS Locked: %s\n", (gtGpsInfo.bGpsLocked) ? "YES" : "NO");
		printf("GPS Lat: %f    Long: %f\n", gtGpsInfo.flat, gtGpsInfo.flon);

	    delay( 200 );
	}

	return 0;
}

//-----------------------------------------------------------------------------------
void setup()
{
	char id_str[5];

    LED_ON;

	//-----------------------
	printf("WireingPi ... ");
	wiringPiSetup();
	printf("OK\n");

	//-----------------------
	printf("I/O Pins ... ");
	
    pinMode(LED_PIN, OUTPUT);

	printf("OK\n");

#if USE_ARDUINO
	//-----------------------
	printf("Arduino ...\n");

	cArduino.Init( ARDUINO_I2C_ADDR );

	printf("\tArduino version: 0x%X\n", cArduino.GetReg( ARDUINO_REG_VERSION ) );

    // Init servos
	printf("Servo Test:\n");
	cArduino.SetReg( ARDUINO_REG_EXTRA_LED, 1 );

	printf("Left ... ");
    SetRudder( RUDDER_FULL_LEFT );
    delay(1000);
	printf("Center ... ");
    SetRudder( RUDDER_CENTER );
    delay(1000);
	printf("Right ... ");
    SetRudder( RUDDER_FULL_RIGHT );
    delay(1000);
	printf("Center ...\n");
    SetRudder( RUDDER_CENTER );
    delay(1000);

	cArduino.SetReg( ARDUINO_REG_EXTRA_LED, 0 );

	printf("OK\n");
#endif	// USE_ARDUINO

	//-----------------------
	printf("GPS ...\n");

	if ((gSerial_fd = serialOpen ("/dev/ttyAMA0", GPS_BAUD)) < 0)
	{
		fprintf (stderr, "\tUnable to open serial device: %s\n", strerror (errno)) ;
		return;
	}
	else
	{
		printf("\tComm port to GPS opened. GPS Baud: %i\n", GPS_BAUD);
	}

	printf("OK\n");

	//-----------------------
	printf("Accelerometer ...\n");	

	cAccel.init( ADXL345_ADDR_ALT_LOW );
	cAccel.setRangeSetting( 2 );

	printf("OK\n");

	//-----------------------
	printf("Compass ...\n");

	cCompass = HMC5883L(); // init HMC5883
	cCompass.SetScale(1.3); // Set the scale of the compass.
  	cCompass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

/*
    // Init compass class
    cCompass.init(false); // Dont set mode yet, we'll do that later on. 

	memset( id_str, 0, sizeof(id_str) );
	cCompass.getID( id_str );
	printf("Compass ID: %s\n", id_str );

#if USE_COMPASS_CALIBRATION
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
#endif // USE_COMPASS_CALIBRATION

    // Set continuous mode
    cCompass.setMode(0);
*/
	printf("OK\n");
     
    // Navigation state machine init
    geNavState = E_NAV_INIT;

    LED_OFF;
}

//-----------------------------------------------------------------------------------
void loop( void )
{
    // Set LED on
    LED_ON;
    
    // **********************
    // Update cGps Data/Status
    // **********************
    gtGpsInfo.bGpsLocked = UpdateGps( &gtGpsInfo );
    
    // **********************************
    // Navigation State Machine variables
    // **********************************
    static float initial_dist_to_waypoint;
    float bearing_tolerance;
    static U8 update_counter = 0;  // 100ms x 10 == 1 second update since loop runs about every 100ms
    static S16 gps_delay = 0;
	static E_NAV_STATE last_NavState = E_NAV_MAX;
    
	// **********************
	// Update compass heading
	// **********************
      
//      if( gtGpsInfo.fmph > 3.0 )
//      {
//        current_heading = gtGpsInfo.fcourse;
//      }
//      else
	{    
		gtNavInfo.current_heading = GetCompassHeading( MAG_VAR );
	}
      
	if( last_NavState != geNavState )
	{
		last_NavState = geNavState;
		PrintProgramState( geNavState );
	}

	// ******************
	// Main State Machine
	// ******************
      switch( geNavState )
      {
      case E_NAV_INIT:
          // Initialize wheels, motors, rudder, comms, etc.

          // Set initial way point target
          gTargetWP = 0;
          geNavState = E_NAV_WAIT_FOR_GPS_LOCK;
          break;
          
      case E_NAV_WAIT_FOR_GPS_LOCK:
          if( gtGpsInfo.bGpsLocked )
          {          
              gps_delay = GPS_STABALIZE_LOCK_TIME;
              geNavState = E_NAV_WAIT_FOR_GPS_STABLIZE;
          }
          break;
          
      case E_NAV_WAIT_FOR_GPS_STABLIZE:

          if( gps_delay-- )
          {
              delay(1000);
          }
          else
          {
#if !USE_HOME_POSITION
              // Save current GPS location as the "Home" waypoint
              gtWayPoint[0].flat = gtGpsInfo.flat;
              gtWayPoint[0].flon = gtGpsInfo.flon;
#endif
#if DO_GPS_TEST
              geNavState = E_NAV_IDLE;
#else
              geNavState = E_NAV_SET_NEXT_WAYPOINT;
#endif
          }
          break;
          
      case E_NAV_SET_NEXT_WAYPOINT:
          gTargetWP++;
          gTargetWP = gTargetWP % NUM_WAY_POINTS;
          
          // Calculate inital bearing to waypoint
          gtNavInfo.bear_to_waypoint = cGps.course_to( gtGpsInfo.flat, gtGpsInfo.flon,
          				    gtWayPoint[gTargetWP].flat, gtWayPoint[gTargetWP].flon );

		  gtNavInfo.dist_to_waypoint = cGps.distance_between( gtGpsInfo.flat, gtGpsInfo.flon,
   						    gtWayPoint[gTargetWP].flat, gtWayPoint[gTargetWP].flon );

          geNavState = E_NAV_START;
          break;
          
      case E_NAV_WAIT_FOR_GPS_RELOCK:
          // Resume navigation if the cGps obtains a lock again
          if( gtGpsInfo.bGpsLocked )
          {          
              geNavState = E_NAV_START;
          }
          break;
          
      case E_NAV_START:
          // Use motors, rudder and compass to turn towards new waypoint
          
          // Which way to turn?
          switch( DirectionToBearing( gtNavInfo.bear_to_waypoint, gtNavInfo.current_heading, DEGREES_TO_BEARING_TOLERANCE ) )
          {
            case E_GO_LEFT:
              printf("Go LEFT\n");
              SetRudder( RUDDER_FULL_LEFT );
              SetSpeed( SPEED_25_PERCENT );
              delay(100);
//              SetRudder( RUDDER_CENTER );
//              SetSpeed( SPEED_STOP );
              break;
            case E_GO_RIGHT:         
              printf("Go RIGHT\n");
              SetRudder( RUDDER_FULL_RIGHT );
              SetSpeed( SPEED_25_PERCENT );
              delay(100);
//              SetRudder( RUDDER_CENTER );
//              SetSpeed( SPEED_STOP );
              break;
            case E_GO_STRAIGHT:
              printf("Go STRAIGHT\n");
              geNavState = E_NAV_RUN;
              SetRudder( RUDDER_CENTER );
              SetSpeed( SPEED_50_PERCENT );

              // Calculate initial distance to next point
              initial_dist_to_waypoint = cGps.distance_between( gtGpsInfo.flat, gtGpsInfo.flon,
    						       gtWayPoint[gTargetWP].flat, gtWayPoint[gTargetWP].flon );
              gtNavInfo.dist_to_waypoint = initial_dist_to_waypoint;
              printf("Distance to waypoint: %f\n", gtNavInfo.dist_to_waypoint);
              break;
          }
          break;
          
      case E_NAV_RUN:
          if( 0 == (update_counter++ % 10) )
          {
              // Update range and bearing to waypoint
              gtNavInfo.dist_to_waypoint = cGps.distance_between( gtGpsInfo.flat, gtGpsInfo.flon,
    						     gtWayPoint[gTargetWP].flat, gtWayPoint[gTargetWP].flon );
    
              gtNavInfo.bear_to_waypoint = cGps.course_to( gtGpsInfo.flat, gtGpsInfo.flon,
              				     gtWayPoint[gTargetWP].flat, gtWayPoint[gTargetWP].flon );
          }
          
          // Is GPS still locked?
          if( false == gtGpsInfo.bGpsLocked )
          {            
              geNavState = E_NAV_STOP;
              break;   
          }
          
          // Adjust bearing to target tolerance for more refined direction pointing
          if( gtNavInfo.dist_to_waypoint <= (initial_dist_to_waypoint * 0.10) )
          {
            bearing_tolerance = DEGREES_TO_BEARING_TOLERANCE * 0.5;
          }
          else
          {
            bearing_tolerance = DEGREES_TO_BEARING_TOLERANCE;
          }
                   
          // Correct track to waypoint (if needed)
          switch( DirectionToBearing( gtNavInfo.bear_to_waypoint, gtNavInfo.current_heading, bearing_tolerance ) )
          {
            case E_GO_LEFT:           
              SetRudder( RUDDER_LEFT );
              break;
            case E_GO_RIGHT:             
              SetRudder( RUDDER_RIGHT );
              break;
            case E_GO_STRAIGHT:            
              SetRudder( RUDDER_CENTER );
              SetSpeed( SPEED_100_PERCENT );
              break;
          }
          
          // Are we there yet?
          if( gtNavInfo.dist_to_waypoint <= SWITCH_WAYPOINT_DISTANCE )
          {
              SetSpeed( SPEED_STOP );
              geNavState = E_NAV_SET_NEXT_WAYPOINT;
          }
          break;
          
      case E_NAV_STOP:
          // Stop navigation and wait to resume
          SetSpeed( SPEED_STOP );
          
          if( false == gtGpsInfo.bGpsLocked )
          {
              geNavState = E_NAV_WAIT_FOR_GPS_RELOCK;
              break;   
          }
          else
          {
              geNavState = E_NAV_IDLE;
          }
          break;
  
      case E_NAV_IDLE:
          // Wait for some external condition to restart us ... i.e. message from RPi, button push, etc.
          // TBD
          break;
      }
    
    // set the LED off
    LED_OFF;
}

//-----------------------------------------------------------------------------------
// Gradually sets the new ESC speed setting unless its STOP
// Assumes LOWER settings == faster
void SetSpeed( int new_setting )
{
  int last_setting;
  int step_and_dir;

	cArduino.SetReg( ARDUINO_REG_ESC, new_setting);
  
/*
  
  if( new_setting == SPEED_STOP )
  {
    gEscServo.write( SPEED_STOP );
    return;
  }
  
  // Get the last value written to the servo
  last_setting = gEscServo.read();
  
   // Which direction to go?
  step_and_dir = (new_setting > last_setting) ? SPEED_STEP_SIZE : SPEED_STEP_SIZE * -1;

  // Move to new setting gradually
  for( ; (step_and_dir > 0 ) ? (last_setting < new_setting) : (last_setting > new_setting); last_setting += step_and_dir )
  {
    if( last_setting > SPEED_BACKUP )  // don't let servo setting go negative!
    {
      break;
    }
    gEscServo.write(last_setting);
    delay(SPEED_STEP_DELAY);
  }
*/
}

//-----------------------------------------------------------------------------------
// Gradually sets the new rudder position
// Assums Right == Higher setting, Left == Lower setting
void SetRudder( int new_setting )
{
	int last_setting;
	int step_and_dir;
  
#if RUDDER_REVERSE
	new_setting = 180 - new_setting;
#endif

	cArduino.SetReg( ARDUINO_REG_STEERING, new_setting);
	return;
/*  
  // Get the last value written to the servo
  last_setting = gRudderServo.read();
  
  // Which direction to go?
  step_and_dir = (new_setting > last_setting) ? RUDDER_STEP_SIZE : RUDDER_STEP_SIZE * -1;

  // Move to new setting gradually
  for( ; (step_and_dir > 0 ) ? (last_setting < new_setting) : (last_setting > new_setting); last_setting += step_and_dir )
  {
    if( last_setting < 0 )  // don't let servo setting go negative!
    {
      break;
    }
    gRudderServo.write(last_setting);
    delay(RUDDER_STEP_DELAY);
  }
  
  // SoftSerial turns off interrupts and screws up the Servo lib! Need to detach!
//  gRudderServo.detach();
*/
}

//-----------------------------------------------------------------------------------
// Returns valid cGps data if GPS has a Fix
bool UpdateGps( tGPS_INFO *ptGpsInfo )
{
    static bool bLocked = false;
    bool bNewGpsData = false;
    unsigned long fix_age;
    int year;
    U8 month, day, hundredths;
	
    // *******************************    
    // Grab GPS data from serial input
    // *******************************
	while( serialDataAvail(gSerial_fd) )
    {
        char c;
		c = serialGetchar(gSerial_fd);

#if DO_GPS_TEST        
        printf("%c", c);
		fflush( stdout );
#endif
        if (cGps.encode(c))
        {
            bNewGpsData = true;
        }
    }
    
    // ********************
    // Process new cGps info
    // ********************
    if( bNewGpsData )
    {        
        // GPS Position
        // retrieves +/- lat/long in 100000ths of a degree
        cGps.f_get_position( &ptGpsInfo->flat, &ptGpsInfo->flon, &fix_age);
        if (fix_age == TinyGPS::GPS_INVALID_AGE)
        {
            bLocked = false;
        }
        else
        {
            bLocked = true;
        }
            
#if USE_GPS_TIME_INFO
        // GPS Time
        cGps.crack_datetime(&year, &month, &day, &ptGpsInfo->hour, &ptGpsInfo->minute, &ptGpsInfo->second, &hundredths, &fix_age);
#endif // USE_GPS_TIME_INFO

        // GPS Speed
        ptGpsInfo->fmph = cGps.f_speed_mph(); // speed in miles/hr
        // course in 100ths of a degree
        ptGpsInfo->fcourse = cGps.f_course();
    }
    
    return bLocked;
}

//-----------------------------------------------------------------------------------
E_DIRECTION DirectionToBearing( float DestinationBearing, float CurrentBearing, float BearingTolerance )
{
	E_DIRECTION eDirToGo;
	float Diff = DestinationBearing - CurrentBearing;
	float AbsDiff = abs(Diff);
	bool bNeg = (Diff < 0);
	bool bBig = (AbsDiff > 180.0);

	if( AbsDiff <= BearingTolerance )
	{
		// We're with-in a few degrees of the target. Just go straight!
		eDirToGo = E_GO_STRAIGHT;
	}
	else
	{
		if( !bNeg && !bBig )
		  eDirToGo = E_GO_RIGHT;
		if( !bNeg && bBig )
		  eDirToGo = E_GO_LEFT;
		if( bNeg && !bBig )
		  eDirToGo = E_GO_LEFT;
		if( bNeg && bBig )
		  eDirToGo = E_GO_RIGHT;
	}

	return eDirToGo;
}

//-----------------------------------------------------------------------------------
float AngleCorrect(float inangle)
{
	float outangle = 0;

	outangle = inangle;

	while( outangle > 360)
	{
		outangle -= 360.0;
	}

	while (outangle < 0)
	{
		outangle += 360.0;
	}

	return(outangle);
}

//-----------------------------------------------------------------------------------
float GetCompassHeading( float declination )
{
	float heading;

	// Retrive the raw values from the compass (not scaled).
	MagnetometerRaw raw = cCompass.ReadRawAxis();

	// Retrived the scaled values from the compass (scaled to the configured scale).
	MagnetometerScaled scaled = cCompass.ReadScaledAxis();

	printf("Compass Raw: x= %i, y= %i, z= %i\n", raw.XAxis, raw.YAxis, raw.ZAxis);
	printf("Compass Scaled: x= %f, y= %f, z= %f\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);

#if USE_COMPASS_TILT_COMPENSATION

	float cx, cy, cz;	// compass values
	int ax, ay, az;	// accel values
	
	static int last_cx, last_cy, last_cz;
	static int last_ax, last_ay, last_az;

	float xh, yh, ayf, axf;

	// Get compass values	
	cx = scaled.XAxis;
	cy = scaled.YAxis;
	cz = scaled.ZAxis;

	m.x = cx;
	m.y = cy;
	m.z = cz;
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

//	cx = TOOLS_LowPassFilter( last_cx, cx );
//	cy = TOOLS_LowPassFilter( last_cy, cy );
//	cz = TOOLS_LowPassFilter( last_cz, cz );

	// Get accelerometer values
	float axyz[3];
	cAccel.get_Gxyz( axyz );
//	cAccel.readAccel( &ax, &ay, &az );

//	ax = TOOLS_LowPassFilter( last_ax, ax );
//	ay = TOOLS_LowPassFilter( last_ay, ay );
//	az = TOOLS_LowPassFilter( last_az, az );

	printf("Accel Raw: x= %f, y= %f, z= %f\n", axyz[0], axyz[1], axyz[2] );

	a.x = axyz[0];
	a.y = axyz[1];
	a.z = axyz[2];

	heading = get_heading((vector){0,-1,0});
 
#if 0
	// from web, doesn't seem to work
	// Convert to rad
	axf = radians( ax );
	ayf = radians( ay );

	xh = cx * cos(ayf) + cy * sin(ayf) * sin(axf) - cz * cos(axf) * sin(ayf);
	yh = cy * cos(axf) + cz * sin(axf);

//	heading = atan2((double)yh,(double)xh) * (180 / PI) -90; // angle in degrees
	heading = atan2( yh, xh );
#endif

#else
   
	// Calculate heading when the magnetometer is level, then correct for signs of axis.
//	heading = atan2(scaled.YAxis, scaled.XAxis);

#if 0
	// Rob's stuff (not sure how or why to use it?)
	float FMagnitude = sqrt( (scaled.YAxis * scaled.YAxis) + (scaled.XAxis * scaled.XAxis) );
	float inclination = atan2(scaled.ZAxis, FMagnitude);
	float inclinationDegrees = AngleCorrect( (inclination * 180 / M_PI) + 57.45);
	float Magnitude = sqrt( (scaled.YAxis * scaled.YAxis) + (scaled.XAxis * scaled.XAxis) + (scaled.ZAxis * scaled.ZAxis) );
	float headingDegrees = AngleCorrect( (heading * 180 / M_PI) + declination );

	printf("inclinationDegrees: %f\n", inclinationDegrees);
	printf("Magnitude: %f\n", Magnitude);
	printf("headingDegrees: %f\n", headingDegrees);

	// end Rob's stuff
#endif

#endif // USE_COMPASS_TILT_COMPENSATION

    // If you have an EAST declination, use += declinationAngle, if you
    // have a WEST declination, use -= declinationAngle 

//	heading += radians( declination );
	heading += declination;

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

    // Convert radians to degrees for readability.
//    return degrees( heading );
	return heading;
}

// Returns the number of degrees from the From vector projected into
// the horizontal plane is away from north.
// 
// Description of heading algorithm: 
// Shift and scale the magnetic reading based on calibration data to
// to find the North vector. Use the acceleration readings to
// determine the Down vector. The cross product of North and Down
// vectors is East. The vectors East and North form a basis for the
// horizontal plane. The From vector is projected into the horizontal
// plane and the angle between the projected vector and north is
// returned.
int get_heading(vector from)
{
    // shift and scale
    m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
    m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
    m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

    vector temp_a = a;
    // normalize
    vector_normalize(&temp_a);
    //vector_normalize(&m);

    // compute E and N
    vector E;
    vector N;
    vector_cross(&m, &temp_a, &E);
    vector_normalize(&E);
    vector_cross(&temp_a, &E, &N);
	
    // compute heading
    int heading = round(atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI);
    if (heading < 0) heading += 360;
	return heading;
}

void vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}


//-----------------------------------------------------------------------------------
void PrintProgramState( E_NAV_STATE eState )
{
	printf("State: ");
	switch( eState )
	{
	case E_NAV_INIT:
	  printf("Init");
	  break;
	case E_NAV_WAIT_FOR_GPS_LOCK:
	  printf("Wait for GPS Lock");
	  break;
	case E_NAV_WAIT_FOR_GPS_STABLIZE:
	  printf("Wait for GPS to Stabalize");
	  break;
	case E_NAV_WAIT_FOR_GPS_RELOCK:
	  printf("Wait for GPS Relock");
	  break;
	case E_NAV_SET_NEXT_WAYPOINT:
	  printf("Set Next Waypoint");
	  break;
	case E_NAV_START:
	  printf("Start");
	  break;
	case E_NAV_RUN:
	  printf("Run");
	  break;
	case E_NAV_STOP:
	  printf("Stop");
	  break;
	case E_NAV_IDLE:
	  printf("Idle");
	  break;
	}
	printf("\n");
	//fflush (stdout);
}

