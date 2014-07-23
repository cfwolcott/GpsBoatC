// **************************************************************
// GPS Boat
// main.c
// **************************************************************

#include <stdio.h>
#include "config.h" // defines I/O pins, operational parameters, etc.

//---------------------------------------------------------------
// local defines

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
    byte hour;
    byte minute;
    byte second;
} tGPS_INFO;

typedef struct
{
    float flat;
    float flon;
} tWAY_POINT;

//---------------------------------------------------------------
// local data

// Global program state
E_NAV_STATE geNavState;

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

boolean UpdateGps( tGPS_INFO *ptGpsInfo );
void    PrintProgramState( E_NAV_STATE eState );
E_DIRECTION DirectionToBearing( float DestinationBearing, float CurrentBearing, float BearingTolerance );
void    SetSpeed( int new_speed );
void    SetRudder( int new_setting );
float   GetCompassHeading( float declination );
void    PlaySong( tSONG *pSong );

//---------------------------------------------------------------
// main
//---------------------------------------------------------------
void main( void )
{
  printf("GpsBoat - Version 1.0\n\n");
  
  printf("Initializing:\n");
  geState = E_STATE_INIT;
  
  //-----------------------
  printf("WireingPi ... ");
  
  printf("OK\n");
  

  //-----------------------
  printf("Arduino ... ");
  
  printf("OK\n");


  //-----------------------
  printf("Compass ... ");
  
  printf("OK\n");
  

  //-----------------------
  printf("GPS ... ");
  
  printf("OK\n");
  
  //-----------------------
  // Main Loop
  //-----------------------
  
  while(1)
  {
    loop();
  }
}

//-----------------------------------------------------------------------------------
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    
    // start Serial for GPS
#if USE_ULTIMATE_GPS
    Serial.begin(9600);
#else
    Serial.begin(4800);
#endif

#if USE_SOFT_SERIAL
    SoftSer.begin(9600);
#endif

    Wire.begin();

    // Compass
    LED_ON;
    PRINT("Compass Calibration...");
    // no delay needed as we have already a delay(5) in HMC5843::init()
    gCompass.init(false); // Dont set mode yet, we'll do that later on. 
    // Calibrate HMC using self test, not recommended to change the gain after calibration.
    gCompass.calibrate(1, 32); // Use gain 1=default, valid 0-7, 7 not recommended.
    // Single mode conversion was used in calibration, now set continuous mode
    gCompass.setMode(0);
    PRINTLN("done");
    LED_OFF;
   
    // Init servos
    gEscServo.attach( SERVO_ESC_PIN );
    gEscServo.write( SPEED_STOP );
    
    gRudderServo.attach( SERVO_RUDDER_PIN );
    gRudderServo.write( RUDDER_CENTER );
    //gRudderServo.detach();
    
    SetRudder( RUDDER_FULL_LEFT );
    PlaySong( &tSOUND_WAYPOINT_START_NAV );
    delay(1000);
    SetRudder( RUDDER_CENTER );
    PlaySong( &tSOUND_WAYPOINT_FOUND );
    delay(1000);
    SetRudder( RUDDER_FULL_RIGHT );
    PlaySong( &tSOUND_SEARCHING );
    delay(1000);
    SetRudder( RUDDER_CENTER );
    PlaySong( &tSOUND_SEARCHING );
    delay(1000);
    
    // Navigation state machine init
    geNavState = E_NAV_INIT;
}

//-----------------------------------------------------------------------------------
void loop( void )
{
    // Set LED on
    LED_ON;
    
    // **********************
    // Update Gps Data/Status
    // **********************
    boolean bGpsLocked;
    bGpsLocked = UpdateGps( &gtGpsData );
    
    // **********************************
    // Navigation State Machine variables
    // **********************************
    static float dist_to_waypoint;
    static float initial_dist_to_waypoint;
    static float bear_to_waypoint;
    float bearing_tolerance;
    static byte update_counter = 0;  // 100ms x 10 == 1 second update since loop runs about every 100ms
    static int gps_delay = 0;
    
      // **********************
      // Update compass heading
      // **********************
      float current_heading;
      
//      if( gtGpsData.fmph > 3.0 )
//      {
//        current_heading = gtGpsData.fcourse;
//      }
//      else
      {    
        current_heading = GetCompassHeading( MAG_VAR );
      }
      
      PRINTLN(current_heading);

      // ******************
      // Main State Machine
      // ******************
      PrintProgramState( geNavState );
      switch( geNavState )
      {
      case E_NAV_INIT:
          // Initialize wheels, motors, rudder, comms, etc.

          // Set initial way point target
          gTargetWP = 0;
          geNavState = E_NAV_WAIT_FOR_GPS_LOCK;
          break;
          
      case E_NAV_WAIT_FOR_GPS_LOCK:
          if( bGpsLocked )
          {          
              gps_delay = GPS_STABALIZE_LOCK_TIME;
              geNavState = E_NAV_WAIT_FOR_GPS_STABLIZE;
          }
          break;
          
      case E_NAV_WAIT_FOR_GPS_STABLIZE:

          if( gps_delay-- )
          {
              PlaySong( &tSOUND_SEARCHING );
              delay(1000);
          }
          else
          {
#if !USE_HOME_POSITION
              // Save current GPS location as the "Home" waypoint
              gtWayPoint[0].flat = gtGpsData.flat;
              gtWayPoint[0].flon = gtGpsData.flon;
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
          bear_to_waypoint = Gps.course_to( gtGpsData.flat, gtGpsData.flon,
          				    gtWayPoint[gTargetWP].flat, gtWayPoint[gTargetWP].flon );
          PRINT("Bearing to waypoint: "); PRINTLN(bear_to_waypoint);
          geNavState = E_NAV_START;
          break;
          
      case E_NAV_WAIT_FOR_GPS_RELOCK:
          // Resume navigation if the Gps obtains a lock again
          if( bGpsLocked )
          {          
              geNavState = E_NAV_START;
          }
          break;
          
      case E_NAV_START:
          PlaySong( &tSOUND_SEARCHING );
          // Use motors, rudder and compass to turn towards new waypoint
          
          // Which way to turn?
          switch( DirectionToBearing( bear_to_waypoint, current_heading, DEGREES_TO_BEARING_TOLERANCE ) )
          {
            case E_GO_LEFT:
              PRINTLN("Go LEFT");
              SetRudder( RUDDER_FULL_LEFT );
              SetSpeed( SPEED_25_PERCENT );
              delay(100);
//              SetRudder( RUDDER_CENTER );
//              SetSpeed( SPEED_STOP );
              break;
            case E_GO_RIGHT:         
              PRINTLN("Go RIGHT");
              SetRudder( RUDDER_FULL_RIGHT );
              SetSpeed( SPEED_25_PERCENT );
              delay(100);
//              SetRudder( RUDDER_CENTER );
//              SetSpeed( SPEED_STOP );
              break;
            case E_GO_STRAIGHT:
              PRINTLN("Go STRAIGHT");
              geNavState = E_NAV_RUN;
              SetRudder( RUDDER_CENTER );
              SetSpeed( SPEED_50_PERCENT );
              // Calculate initial distance to next point
              initial_dist_to_waypoint = Gps.distance_between( gtGpsData.flat, gtGpsData.flon,
    						       gtWayPoint[gTargetWP].flat, gtWayPoint[gTargetWP].flon );
              dist_to_waypoint = initial_dist_to_waypoint;
              PRINT("Distance to waypoint: "); PRINTLN(dist_to_waypoint);
              PlaySong( &tSOUND_WAYPOINT_START_NAV );
              break;
          }
          break;
          
      case E_NAV_RUN:
          if( 0 == (update_counter++ % 10) )
          {
              // Update range and bearing to waypoint
              dist_to_waypoint = Gps.distance_between( gtGpsData.flat, gtGpsData.flon,
    						       gtWayPoint[gTargetWP].flat, gtWayPoint[gTargetWP].flon );
    
              bear_to_waypoint = Gps.course_to( gtGpsData.flat, gtGpsData.flon,
              				        gtWayPoint[gTargetWP].flat, gtWayPoint[gTargetWP].flon );           
          }
          
          // Is GPS still locked?
          if( false == bGpsLocked )
          {            
              geNavState = E_NAV_STOP;
              break;   
          }
          
          // Adjust bearing to target tolerance for more refined direction pointing
          if( dist_to_waypoint <= (initial_dist_to_waypoint * 0.10) )
          {
            bearing_tolerance = DEGREES_TO_BEARING_TOLERANCE * 0.5;
          }
          else
          {
            bearing_tolerance = DEGREES_TO_BEARING_TOLERANCE;
          }
                   
          // Correct track to waypoint (if needed)
          switch( DirectionToBearing( bear_to_waypoint, current_heading, bearing_tolerance ) )
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
          if( dist_to_waypoint <= SWITCH_WAYPOINT_DISTANCE )
          {
              SetSpeed( SPEED_STOP );
              PlaySong( &tSOUND_WAYPOINT_FOUND );
              geNavState = E_NAV_SET_NEXT_WAYPOINT;
          }
          break;
          
      case E_NAV_STOP:
          // Stop navigation and wait to resume
          SetSpeed( SPEED_STOP );
          
          if( false == bGpsLocked )
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
    
    delay(100);
}

//-----------------------------------------------------------------------------------
// Gradually sets the new ESC speed setting unless its STOP
// Assumes LOWER settings == faster
void SetSpeed( int new_setting )
{
  int last_setting;
  int step_and_dir;
  
  return;
  
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

  gRudderServo.write(new_setting);
  return;
  
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
}

//-----------------------------------------------------------------------------------
// Returns valid Gps data if GPS has a Fix
boolean UpdateGps( tGPS_INFO *ptGpsInfo )
{
    static boolean bLocked = false;
    boolean bNewGpsData = false;
    unsigned long fix_age;
    int year;
    byte month, day, hundredths;
	
    // *******************************    
    // Grab GPS data from serial input
    // *******************************
#if USE_SOFT_SERIAL
    while (SoftSer.available())
    {
        int c = SoftSer.read();
#else
    while (Serial.available())
    {
        int c = Serial.read();
#endif

#if DO_GPS_TEST        
        Serial.print((char)c);
#endif
        if (Gps.encode(c))
        {
            bNewGpsData = true;
        }
    }
    
    // ********************
    // Process new Gps info
    // ********************
    if( bNewGpsData )
    {        
        // GPS Position
        // retrieves +/- lat/long in 100000ths of a degree
        Gps.f_get_position( &ptGpsInfo->flat, &ptGpsInfo->flon, &fix_age);
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
        Gps.crack_datetime(&year, &month, &day, &ptGpsInfo->hour, &ptGpsInfo->minute, &ptGpsInfo->second, &hundredths, &fix_age);
#endif // USE_GPS_TIME_INFO

        // GPS Speed
        ptGpsInfo->fmph = Gps.f_speed_mph(); // speed in miles/hr
        // course in 100ths of a degree
        ptGpsInfo->fcourse = Gps.f_course();
    }
    
    return bLocked;
}

//-----------------------------------------------------------------------------------
E_DIRECTION DirectionToBearing( float DestinationBearing, float CurrentBearing, float BearingTolerance )
{
  E_DIRECTION eDirToGo;
  float Diff = DestinationBearing - CurrentBearing;
  float AbsDiff = abs(Diff);
  boolean bNeg = (Diff < 0);
  boolean bBig = (AbsDiff > 180.0);
  
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
float GetCompassHeading( float declination )
{
    // Calculate heading when the magnetometer is level, then correctfor signs of axis. 
    float mag_x, mag_y, mag_z;
    
    gCompass.getValues( &mag_x, &mag_y, &mag_z );
    
    float heading = atan2( mag_y, mag_x );

    // If you have an EAST declination, use += declinationAngle, if you
    // have a WEST declination, use -= declinationAngle 

    heading -= radians(declination);

    // Correct for when signs are reversed. 
    if(heading < 0)
    {
//	heading += TWO_PI;
      heading += 2 * M_PI;
    }

    // Check for wrap due to addition of declination. 
//    if(heading > TWO_PI)
//    {
//    	heading -= TWO_PI;
//    }

    // Convert radians to degrees for readability.
//    return degrees( heading );
    return (heading * 180/M_PI);
}

//-----------------------------------------------------------------------------------
void PrintProgramState( E_NAV_STATE eState )
{
  PRINT("State: ");
  switch( eState )
  {
    case E_NAV_INIT:
      PRINTLN("Init");
      break;
    case E_NAV_WAIT_FOR_GPS_LOCK:
      PRINTLN("Wait for GPS Lock");
      break;
    case E_NAV_WAIT_FOR_GPS_STABLIZE:
      PRINTLN("Wait for GPS to Stabalize");
      break;
    case E_NAV_WAIT_FOR_GPS_RELOCK:
      PRINTLN("Wait for GPS Relock");
      break;
    case E_NAV_SET_NEXT_WAYPOINT:
      PRINTLN("Set Next Waypoint");
      break;
    case E_NAV_START:
      PRINTLN("Start");
      break;
    case E_NAV_RUN:
      PRINTLN("Run");
      break;
    case E_NAV_STOP:
      PRINTLN("Stop");
      break;
    case E_NAV_IDLE:
      PRINTLN("Idle");
      break;
  }
}
