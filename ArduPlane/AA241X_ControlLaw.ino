
#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "PID_Controller.h"
#include "HeadingController.h"

/**** Helper Function Prototypes ****/
static void       Limit(float &variable, float maximum, float minimum);

static int8_t     my_signed_8_bit_variable = 10;  // integer numbers between -128 and 127
static uint8_t    my_unsigned_8_bit_variable = 10;  // positive integer numbers between 0 and 255

static int16_t    my_signed_16_bit_variable = 10;  // integer numbers between −32768 and 32767
static uint16_t   my_unsigned_16_bit_variable = 10;  // positive integer numbers between 0 and 65535

static int32_t    my_signed_32_bit_variable = 10;  // integer numbers between −2147483648 and 2147483647
static uint32_t   my_unsigned_32_bit_variable = 10;  // positive integer numbers between 0 and 4294967295

/**** Control Mode ****/
static uint32_t   ROLL_STABILIZE_MODE = 1;
static uint32_t   STABILIZE_MODE = 2;
static uint32_t   HEADING_HOLD_MODE = 3;
static uint32_t   FBW_MODE = 4;
static uint32_t   ATT_HOLD = 5;
static uint32_t   WAYPOINT_NAV = 6;
static uint32_t   controlMode = 1; // Determine automatic control mode

/**** Mission Plan ****/
static uint32_t   missionPlan = 1;  // Determine the mission type
static uint32_t   STATIC_ROUTE_1 = 1;
static uint32_t   STATIC_ROUTE_2 = 2;
static uint32_t   STATIC_ROUTE_3 = 3;
static uint32_t   STATIC_ROUTE_4 = 4;

static uint32_t   DYNAMIC_ROUTE_1 = 1;
static uint32_t   DYNAMIC_ROUTE_2 = 2;

/**** Time Variables ****/
static uint32_t   numCalls    = 0;    // Number of times the AUTO loop has been called
static float      delta_t_avg = 0; // Average value of delta_t
static float      delta_t_sum = 0; // Total sum of time since start of AUTO loop

/**** State Variables ****/
static float altitudeCommand  = 50;   // 50 meters is default altitude
static float airspeedCommand  = 9;   // 9 meters / second is default airspeed
static float headingCommand   = 0;    // Go North
static float rollCommand      = 0;    // Keep Level
static float pitchCommand     = (THETA_COMMAND/180.0)*PI; //.075; // 4.5 degrees pitch

/*************************** Mechanical Limit Variables ***********************************************

Notes about the PWM signal commands, there doesn't seem to be a more appropriate place to put these
comments in anywhere but here at this point. The commanded signal to control the PWM signals going to
each of the servos is produced here in this loop. The command is given in values of percentage, which
are ultimately translated into the PWM duty cycle by the existing StarduPilot code. The chXout variables
given in the "Status" window of the Mission Planner are mapped linearly from the percentages commanded
in the AUTO_FastLoop function.

For each output, the 0% (0 command) is mapped to a corresponding value of 800 on the chXout menu. The
100% (100 command) is mapped to a corresponding value of 2200 on the chXmenu. The output of the menu
is cut of at 900 and 2100 for reasons unknown, but the middle value is 1500 and for each integer increase
in the value of the commanded percentage, a value of 14 is added to the chXout value.

Each control surface has mechanical limits that should not be exceeded by either the RC controller (pilot
input) or the automatic flight system. These mechanical limits are what is defined in this section.

--------------------------------------- Bixler 2 Mechanical Limits ----------------------------------*/
static float pitchMax  = 90; // Elevator Down
static float pitchMin  = 20;  // Elevator Up
static float rollMax   = 90; // Roll Left Aileron
static float rollMin   = 10;   // Roll Right Aileron
static float rudderMax = 90; // Left Rudder
static float rudderMin = 10;   // Right Rudder
static float throttleMax = 95; // Throttle Up
static float throttleMin = 20;   // Throttle Down

/*------------------------------------ Navigation Loops Limits -------------------------------------*/
static float bankAngleMax = .52;  // 30 degrees max
static float bankAngleMin = -.52;  // -30 degrees min
static float pitchAngleMax = .21;   // 12 degrees pitch
static float pitchAngleMin = -0.122; // -7 degrees pitch
static float airspeedCommandMax = 15; // 15 meters / second
static float airspeedCommandMin = 5;  // 5 meters / second

/**** PID Loops ****/
PidController rollController241X(RLL_2_SRV_P, // Proportional Gain
                                 RLL_2_SRV_I, // Integral Gain
                                 RLL_2_SRV_D, // Derivative Gain
                                 25,          // Maximum Controller Output
                                 1,           // Maximum Integral Error
                                 3,           // Maximum Derivative Error
                                 5,           // Maximum Integral Term
                                 5);          // Maximum Derivative Term

PidController pitchController241X(PTCH_2_SRV_P, // Proportional Gain
                                  PTCH_2_SRV_I, // Integral Gain
                                  PTCH_2_SRV_D, // Derivative Gain
                                  20,           // Maximum Controller Output
                                  1,            // Maximum Integral Error
                                  3,            // Maximum Derivative Error
                                  5,            // Maximum Integral Term
                                  5);           // Maximum Derivative Term
                                  
PidController rudderController241X(RUD_2_SRV_P,  // Proportional Gain
                                   RUD_2_SRV_I,  // Integral Gain
                                   RUD_2_SRV_D,  // Derivative Gain
                                   20,           // Maximum Controller Output
                                   1,            // Maximum Integral Error
                                   3,            // Maximum Derivative Error
                                   5,            // Maximum Integral Term
                                   5);           // Maximum Derivative Term                                  

PidController airspeedController241X(SPD_2_SRV_P, // Proportional Gain
                                     SPD_2_SRV_I, // Integral Gain
                                     SPD_2_SRV_D, // Derivative Gain
                                     20,          // Maximum Controller Output
                                     10,          // Maximum Integral Error
                                     3,           // Maximum Derivative Error
                                     5,           // Maximum Integral Term
                                     5);          // Maximum Derivative Term

HeadingController headingController241X(HEAD_2_SRV_P, // Proportional Gain
                                        HEAD_2_SRV_I, // Integral Gain
                                        HEAD_2_SRV_D, // Derivative Gain
                                        25,           // Maximum Controller Output
                                        1,            // Maximum Integral Error
                                        3,            // Maximum Derivative Error
                                        5,            // Maximum Integral Term
                                        5);           // Maximum Derivative Term
                                    
PidController altitudeController241X(ALT_HOLD_P,  // Proportional Gain
                                     0.0,//ALT_HOLD_I,  // Integral Gain
                                     0.0,//ALT_HOLD_D,  // Derivative Gain
                                     25,           // Maximum Controller Output
                                     1,            // Maximum Integral Error
                                     3,            // Maximum Derivative Error
                                     5,            // Maximum Integral Term
                                     5);           // Maximum Derivative Term                                    
                                 
/**** PID Controller Outputs ****/
// Inner Loop Command Signals, set by default to RC pilot commands
float rollControllerOut = 0;
float pitchControllerOut = 0;
float rudderControllerOut = 0;
  
// Outer Loop Command Signals
float airspeedControllerOut = 0;
float headingControllerOut  = 0;
float altitudeControllerOut = 0;

/**** Store RC Inputs from last run through the fast loop ****/
float RC_pitch_old = 0;
float RC_throttle_old = 0;
float RC_roll_old = 0;
float RC_rudder_old = 0;

/**** Waypoint Navigation ****/
uint32_t Ndim = 2;
uint32_t Nwp = 0;
float **waypoints;

// These functions are executed when control mode is in AUTO
// Please read AA241X_aux.h for all necessary definitions and interfaces

// *****   AA241X Fast Loop - @ ~50Hz   ***** //
static void AA241X_AUTO_FastLoop(void) 
{
  // Time between function calls
  float delta_t = (CPU_time_ms - Last_AUTO_stampTime_ms); // Get delta time between AUTO_FastLoop calls  
  
  // Inner Loop Command Signals, set by default to RC pilot commands: RC_XXXX_Trim gets added back later on
  rollControllerOut = RC_roll - RC_Roll_Trim;
  pitchControllerOut = RC_pitch - RC_Pitch_Trim;
  rudderControllerOut = RC_rudder - RC_Rudder_Trim;
  
  // Outer Loop Command Signals
  airspeedControllerOut = 0;
  headingControllerOut  = 0;
  altitudeControllerOut = 0;

//  static struct snapshot mySnapShot = takeASnapshot();
  
  // Checking if we've just switched to AUTO. If more than 100ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if (delta_t > 100)
  {
    // Reset Average of delta_t
    delta_t_avg = 0;
    delta_t_sum = 0;
    numCalls    = 0;
    
    // Just switched to AUTO, initialize all controller loops
    rollController241X.Initialize(RLL_2_SRV_P, RLL_2_SRV_I, RLL_2_SRV_D);
    pitchController241X.Initialize(PTCH_2_SRV_P, PTCH_2_SRV_I, PTCH_2_SRV_D);
    rudderController241X.Initialize(RUD_2_SRV_P, RUD_2_SRV_I, RUD_2_SRV_D);
    airspeedController241X.Initialize(SPD_2_SRV_P, SPD_2_SRV_I, SPD_2_SRV_D);
    headingController241X.Initialize(HEAD_2_SRV_P, HEAD_2_SRV_I, HEAD_2_SRV_D);
    altitudeController241X.Initialize(ALT_HOLD_P, 0.0 /*ALT_HOLD_I*/ , 0.0 /*ALT_HOLD_D*/ );
    
    // Save all initial settings
    if(gpsOK == true){
      altitudeCommand = -Z_position_GPS;
      altitudeController241X.SetReference(altitudeCommand);
    }else{
      altitudeCommand = -Z_position_Baro;
      altitudeController241X.SetReference(altitudeCommand);
    }
    
    headingCommand = ground_course;
    headingController241X.SetReference(headingCommand);
    
    airspeedCommand = Air_speed;
    airspeedController241X.SetReference(airspeedCommand);
    
    // Determine control mode from bits in parameter list
    if(MODE_SELECT > .5 && MODE_SELECT < 1.5){
      controlMode = ROLL_STABILIZE_MODE;
    }else if(MODE_SELECT > 1.5 && MODE_SELECT < 2.5){
      controlMode = STABILIZE_MODE;
    }else if(MODE_SELECT > 2.5 && MODE_SELECT < 3.5){
      controlMode = HEADING_HOLD_MODE;
    }else if(MODE_SELECT > 3.5 && MODE_SELECT < 4.5){
      controlMode = FBW_MODE;
      
      // Mission Planner based pitch command rather than hard coded up top
      pitchCommand = (THETA_COMMAND/180.0)*PI;
    }else if(MODE_SELECT > 4.5 && MODE_SELECT < 5.5){
      controlMode = ATT_HOLD;
    }else if(MODE_SELECT > 5.5 && MODE_SELECT < 6.5){
      controlMode = WAYPOINT_NAV;
      
      // Mission Planner based pitch command until trim settings determined
      pitchCommand = (THETA_COMMAND/180.0)*PI;
    }
    
    // Set RC old values to trim state to capture deltas
    RC_pitch_old = RC_Pitch_Trim;
    RC_roll_old    = RC_Roll_Trim;
    RC_rudder_old   = RC_Rudder_Trim;
    RC_throttle_old = RC_Throttle_Trim;
    
  }
  
  // Time Related Tracking
  delta_t_sum += delta_t;
  numCalls    += 1;
  delta_t_avg  = delta_t_sum/numCalls;
  
  // Determine Inner Loop Commands Based on Control Mode
  if (controlMode == ROLL_STABILIZE_MODE)
  {
    // Keep Roll angle controlled based on RC pilot input (should be zero when stick is in center)
      rollCommand = (RC_roll-RC_Roll_Trim)*0.01*PI;
      rollController241X.SetReference(rollCommand);
      rollControllerOut = rollController241X.Step(delta_t, roll);
  }else if (controlMode == STABILIZE_MODE)
  {
    // Keep Roll and Pitch angles controlled based on RC pilot input
      // Roll Commands
      rollCommand = (RC_roll-RC_Roll_Trim)*0.01*PI;
      rollController241X.SetReference(rollCommand);
      rollControllerOut = rollController241X.Step(delta_t, roll);
      
      // Pitch Commands
      pitchCommand = (RC_pitch - RC_Pitch_Trim)*0.01*PI/4.0 + (THETA_COMMAND/180.0)*PI;
      pitchController241X.SetReference(pitchCommand);
      pitchControllerOut = pitchController241X.Step(delta_t, pitch);
      
      // Rudder Commands
      // rudderCommand = 
  }else if (controlMode == HEADING_HOLD_MODE)
  {
    // Maintain Heading, RC pilot commands offset from heading that was saved
      // Heading Commands
      if (fabs(RC_roll - RC_Roll_Trim) > 5)
      {
        // Allow breakout room just in case RC_Roll_Trim is not DEAD on
        headingCommand += 0.01*(RC_roll - RC_Roll_Trim)/RC_Roll_Trim; // .0872 rad/s change rate based on 50 Hz
        
        // Check radian range of heading command
        if(headingCommand > 2*PI)
        {
          headingCommand -= 2*PI;
        }else if(headingCommand < 0)
        {
          headingCommand += 2*PI;
        }
        
        headingController241X.SetReference(headingCommand);
      }
      headingControllerOut = headingController241X.Step(delta_t, ground_course);
      Limit(headingControllerOut, bankAngleMax, bankAngleMin);

      // Roll Commands
      rollController241X.SetReference(headingControllerOut);
      rollControllerOut = rollController241X.Step(delta_t, roll); 
      
  }else if (controlMode == FBW_MODE)
  {
    // Maintain heading, altitude, and airspeed RC pilot commands offsets from saved initial conditions
      // Heading Commands
      if ( fabs(RC_roll - RC_Roll_Trim) > 5 )
      {      
        // Allow breakout room just in case RC_Roll_Trim is not DEAD on
        headingCommand += 0.025*(RC_roll - RC_Roll_Trim)/RC_Roll_Trim; // .0872 rad/s change rate based on 50 Hz 0.00174
        
        // Check radian range of heading command
        if(headingCommand > 2*PI)
        {
          headingCommand -= 2*PI;
        }else if(headingCommand < 0)
        {
          headingCommand += 2*PI;
        }
        
        headingController241X.SetReference(headingCommand);
      }
      headingControllerOut = headingController241X.Step(delta_t, ground_course);
      Limit(headingControllerOut, bankAngleMax, bankAngleMin);

      // Roll Commands
      rollController241X.SetReference(headingControllerOut);
      rollControllerOut = rollController241X.Step(delta_t, roll);
      
      // Altitude Commands
      float altitude = 1; // Default altitude
      if(gpsOK == true)
        altitude = -Z_position_GPS;
      else
        altitude = -Z_position_Baro;
      
      if(fabs(RC_pitch - RC_Pitch_Trim) > 5)
      {
        altitudeCommand += 0.04*(RC_pitch - RC_Pitch_Trim)/RC_Pitch_Trim; // 2 m/s change rate based on 50 Hz
        altitudeController241X.SetReference(altitudeCommand);
      }
      altitudeControllerOut = altitudeController241X.Step(delta_t, altitude);
      Limit(altitudeControllerOut, pitchAngleMax, pitchAngleMin);
      
      // Pitch Commands
      pitchController241X.SetReference(altitudeControllerOut);
      pitchControllerOut = pitchController241X.Step(delta_t, pitch);
      
      // Airspeed Commands
      airspeedCommand += 0.1*(RC_throttle - RC_throttle_old);
      RC_throttle_old = RC_throttle;
      Limit(airspeedCommand, airspeedCommandMax, airspeedCommandMin);
      airspeedController241X.SetReference(airspeedCommand);
      airspeedControllerOut = airspeedController241X.Step(delta_t, Air_speed);
  }
  else if (controlMode == ATT_HOLD)
  {
      // Hold Roll Angle
      if ( fabs(RC_roll - RC_Roll_Trim) > 5)
      {      
        // Allow breakout room just in case RC_Roll_Trim is not DEAD on
        rollCommand += 0.017*(RC_roll - RC_Roll_Trim)/RC_Roll_Trim; // .0872 rad/s change rate based on 50 Hz 0.00174
      }
      
      // Roll Commands
      rollController241X.SetReference(rollCommand);
      rollControllerOut = rollController241X.Step(delta_t, roll);
      
      // Hold Pitch Angle
      if ( fabs(RC_pitch - RC_Pitch_Trim) > 5)
      {
        pitchCommand += 0.01*(RC_pitch - RC_Pitch_Trim)/RC_Pitch_Trim; // .0872 rad/s change rate based on 50 Hz 0.00174
      }
      
      // Pitch Commands
      pitchController241X.SetReference(pitchCommand);
      pitchControllerOut = pitchController241X.Step(delta_t, pitch);
  }
  else if (controlMode == WAYPOINT_NAV)
  {
    // This mode requires that headingCommand be updated and within 0 to 2PI
    
    // headingCommand should be updated by the waypoint nav functions called in the medium loop  
    headingController241X.SetReference(headingCommand);
    
    // Get Bank Angle command to track heading
    headingControllerOut = headingController241X.Step(delta_t, ground_course);
    Limit(headingControllerOut, bankAngleMax, bankAngleMin);

    // Roll Commands
    rollController241X.SetReference(headingControllerOut);
    rollControllerOut = rollController241X.Step(delta_t, roll);
    
    // Pitch Commands
    pitchCommand = (RC_pitch - RC_Pitch_Trim)*0.01*PI/4.0 + (THETA_COMMAND/180.0)*PI;
    pitchController241X.SetReference(pitchCommand);
    pitchControllerOut = pitchController241X.Step(delta_t, pitch);
        
  }
  
  // Update Roll Servo Command  
  if(controlMode == ROLL_STABILIZE_MODE || controlMode == STABILIZE_MODE || controlMode == FBW_MODE || controlMode == HEADING_HOLD_MODE || controlMode == ATT_HOLD)
  {
    float rollOut    = RC_Roll_Trim + rollControllerOut;
    Limit(rollOut, rollMax, rollMin);
    Roll_servo       = rollOut;
  }
  else
  {
    Roll_servo       = RC_roll;
  }
  
  // Update Pitch Servo Command
  if(controlMode == STABILIZE_MODE || controlMode == FBW_MODE || controlMode == ATT_HOLD)
  {
    float pitchOut   = RC_Pitch_Trim + pitchControllerOut;
    Limit(pitchOut, pitchMax, pitchMin);
    Pitch_servo      = pitchOut;
  }
  else
  {
    Pitch_servo      = RC_pitch;
  }

  // Update Rudder Servo Command
  if(controlMode == STABILIZE_MODE || controlMode == HEADING_HOLD_MODE || controlMode == FBW_MODE || controlMode == ATT_HOLD)
  {
    float rudderOut  = RC_Rudder_Trim + rudderControllerOut;
    Limit(rudderOut, rudderMax, rudderMin);
    Rudder_servo     = rudderOut;
  } 
  else
  {
    Rudder_servo     = RC_rudder;
  }
  
  // Update Throttle PWM Command
  if(controlMode == FBW_MODE)
  {
    float throttleOut = RC_throttle + airspeedControllerOut;
    Limit(throttleOut, throttleMin, throttleMax);
    Throttle_servo   = RC_throttle + airspeedControllerOut; //throttleOut;
  }
  else
  {
    Throttle_servo   = RC_throttle;
  }  
  

};

// *****   AA241X Medium Loop - @ ~10Hz  *****  //
static void AA241X_AUTO_MediumLoop(void)
{
  // Time between function calls
  float delta_t = (CPU_time_ms - Last_AUTO_stampTime_ms); // Get delta time between AUTO_FastLoop calls  
  
  // Checking if we've just switched to AUTO. If more than 100ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if (delta_t > 100)
  {
    // Clear waypoints
    for (uint32_t i=0; i<Nwp; i++) {
      delete[] waypoints[i]; waypoints[i] = NULL;
    }
    
    // Initialize route from mission planner
    if (routeNumber == STATIC_ROUTE_1) {
      // Specify Waypoints
      Nwp = 3;
      for (uint32_t i=0; i<Nwp; i++) {
        waypoints[i] = new float[Ndim];
      }
      
    
    // Initialize route object and pass waypoints
    
    // Your initialization stuff for the medium frequency loop here
    
  }
  
  // heading = waypoint.step
  
  // Checking if we've just switched to AUTO. If more than 100ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if (delta_t > 100)
  {
    // Just switched to AUTO, initialize all controller loops
    rollController241X.Initialize(RLL_2_SRV_P, RLL_2_SRV_I, RLL_2_SRV_D);
    pitchController241X.Initialize(PTCH_2_SRV_P, PTCH_2_SRV_I, PTCH_2_SRV_D);
    rudderController241X.Initialize(RUD_2_SRV_P, RUD_2_SRV_I, RUD_2_SRV_D);
    airspeedController241X.Initialize(SPD_2_SRV_P, SPD_2_SRV_I, SPD_2_SRV_D);
    headingController241X.Initialize(HEAD_2_SRV_P, HEAD_2_SRV_I, HEAD_2_SRV_D);
    altitudeController241X.Initialize(ALT_HOLD_P, ALT_HOLD_I, ALT_HOLD_D);
    
    // Save all initial settings
    if(gpsOK == true){
      altitudeCommand = -Z_position_GPS;
      altitudeController241X.SetReference(altitudeCommand);
    }else{
      altitudeCommand = -Z_position_Baro;
      altitudeController241X.SetReference(altitudeCommand);
    }
    
    headingCommand = ground_course;
    headingController241X.SetReference(headingCommand);
    
    airspeedCommand = Air_speed;
    airspeedController241X.SetReference(airspeedCommand);
    
    // Determine control mode from bits in parameter list
    if(MODE_SELECT > .5 && MODE_SELECT < 1.5){
      controlMode = ROLL_STABILIZE_MODE;
    }else if(MODE_SELECT > 1.5 && MODE_SELECT < 2.5){
      controlMode = STABILIZE_MODE;
    }else if(MODE_SELECT > 2.5 && MODE_SELECT < 3.5){
      controlMode = HEADING_HOLD_MODE;
    }else if(MODE_SELECT > 3.5 && MODE_SELECT < 4.5){
      controlMode = FBW_MODE;
      
      // Mission Planner based pitch command rather than hard coded up top
      pitchCommand = (THETA_COMMAND/180.0)*PI;
    }else if(MODE_SELECT > 4.5 && MODE_SELECT < 5.5){
      controlMode = ATT_HOLD;
    }else if(MODE_SELECT > 5.5 && MODE_SELECT < 6.5){
      controlMode = WAYPOINT_NAV;
      
      // Mission Planner based pitch command until trim settings determined
      pitchCommand = (THETA_COMMAND/180.0)*PI;
    }
    
    // Set RC old values to trim state to capture deltas
    RC_pitch_old = RC_Pitch_Trim;
    RC_roll_old    = RC_Roll_Trim;
    RC_rudder_old   = RC_Rudder_Trim;
    RC_throttle_old = RC_Throttle_Trim;
    
  }
  
  // YOUR CODE HERE
  if (controlMode == WAYPOINT_NAV)
  {
    headingCommand = 0.0;//Jerry's function
  }
  
};





// *****   AA241X Slow Loop - @ ~1Hz  *****  //
static void AA241X_AUTO_SlowLoop(void)
{  
  controller_summary RollControllerSummary = rollController241X.GetControllerSummary();
  controller_summary PitchControllerSummary = pitchController241X.GetControllerSummary();
  controller_summary HeadingControllerSummary = headingController241X.GetControllerSummary();    
  controller_summary AirspeedControllerSummary = airspeedController241X.GetControllerSummary();    
  
  hal.console->printf_P(PSTR("\n Avg dT: %f \n"), delta_t_avg);
  hal.console->printf_P(PSTR("\n Control Mode: %lu \n"), controlMode);
  
  /*
  // Debug Statements
  hal.console->printf_P(PSTR("\n Roll Value: %f \n"), roll);  
  hal.console->printf_P(PSTR("\n Roll Error: %f \n"), RollControllerSummary.error);
  hal.console->printf_P(PSTR("Roll Integrated Error: %f \n"), RollControllerSummary.i_error);
  hal.console->printf_P(PSTR("Roll Derivative Error: %f \n"), RollControllerSummary.d_error);
  hal.console->printf_P(PSTR("Roll Proportional Term: %f \n"), RollControllerSummary.p_term);
  hal.console->printf_P(PSTR("Roll Integral Term: %f \n"), RollControllerSummary.i_term);
  hal.console->printf_P(PSTR("Roll Derivative Term: %f \n"), RollControllerSummary.d_term);
  hal.console->printf_P(PSTR("Roll Output: %f \n"), RollControllerSummary.output);  

  hal.console->printf_P(PSTR("\n Pitch Value: %f \n"), pitch);
  hal.console->printf_P(PSTR("\n Pitch Command: %f \n"), pitch_command);
  hal.console->printf_P(PSTR("\n Pitch Error: %f \n"), PitchControllerSummary.error);
  hal.console->printf_P(PSTR("Pitch Integrated Error: %f \n"), PitchControllerSummary.i_error);
  hal.console->printf_P(PSTR("Pitch Derivative Error: %f \n"), PitchControllerSummary.d_error);
  hal.console->printf_P(PSTR("Pitch Proportional Term: %f \n"), PitchControllerSummary.p_term);
  hal.console->printf_P(PSTR("Pitch Integral Term: %f \n"), PitchControllerSummary.i_term);
  hal.console->printf_P(PSTR("Pitch Derivative Term: %f \n"), PitchControllerSummary.d_term);
  hal.console->printf_P(PSTR("Pitch Output: %f \n"), PitchControllerSummary.output);
  
  hal.console->printf_P(PSTR("\n RC_roll: %f \n"), RC_roll);
  hal.console->printf_P(PSTR("RC_pitch: %f \n"), RC_pitch);
  hal.console->printf_P(PSTR("RC_rudder: %f \n"), RC_rudder);
  */
  
  /*
  hal.console->printf_P(PSTR("\n Heading Command: %f \n"), headingCommand);
  hal.console->printf_P(PSTR("Current Heading: %f \n"), ground_course);
  hal.console->printf_P(PSTR("Heading Error: %f \n"), HeadingControllerSummary.error);
  hal.console->printf_P(PSTR("Heading Integrated Error: %f \n"), HeadingControllerSummary.i_error);
  hal.console->printf_P(PSTR("Heading Derivative Error: %f \n"), HeadingControllerSummary.d_error);
  hal.console->printf_P(PSTR("Heading Proportional Term: %f \n"), HeadingControllerSummary.p_term);
  hal.console->printf_P(PSTR("Heading Integral Term: %f \n"), HeadingControllerSummary.i_term);
  hal.console->printf_P(PSTR("Heading Derivative Term: %f \n"), HeadingControllerSummary.d_term);
  hal.console->printf_P(PSTR("Heading Output: %f \n"), HeadingControllerSummary.output);
  hal.console->printf_P(PSTR("Bank Angle Command: %f \n"), headingControllerOut);
  hal.console->printf_P(PSTR("Bank Angle Current: %f \n"), roll);  
  */
  
  //hal.console->printf_P(PSTR("\n Airspeed Value: %f \n"), Air_speed);
  //hal.console->printf_P(PSTR("Airspeed Command: %f \n"), airspeedCommand);
  //hal.console->printf_P(PSTR("Airspeed Error: %f \n"), AirspeedControllerSummary.error);
  //hal.console->printf_P(PSTR("Airspeed Integrated Error: %f \n"), AirspeedControllerSummary.i_error);
  //hal.console->printf_P(PSTR("Airspeed Derivative Error: %f \n"), AirspeedControllerSummary.d_error);
  //hal.console->printf_P(PSTR("Airspeed Proportional Term: %f \n"), AirspeedControllerSummary.p_term);
  //hal.console->printf_P(PSTR("Airspeed Integral Term: %f \n"), AirspeedControllerSummary.i_term);
  //hal.console->printf_P(PSTR("Airspeed Derivative Term: %f \n"), AirspeedControllerSummary.d_term);
  //hal.console->printf_P(PSTR("Airspeed Output: %f \n"), AirspeedControllerSummary.output);
  //hal.console->printf_P(PSTR("Throttle Trim: %f \n"), RC_Throttle_Trim);
  //hal.console->printf_P(PSTR("Airspeed Controller Out: %f \n"), airspeedControllerOut);
  
  //hal.console->printf_P(PSTR("Heading Command: %f \n"), headingCommand);
  //hal.console->printf_P(PSTR("Altitude Command: %f \n"), altitudeCommand);
  //hal.console->printf_P(PSTR("Airspeed Command: %f \n"), airspeedCommand);  
  //hal.console->printf_P(PSTR("fabs(RC_roll - RC_Roll_Trim): %f \n"), fabs(RC_roll - RC_Roll_Trim) );
  hal.console->printf_P(PSTR("pitchCommand: %f \n"), pitchCommand);
  hal.console->printf_P(PSTR("rollCommand: %f \n"), rollCommand);
};

/**** Limit function to not exceed mechanical limits of the servos ****/
static void Limit(float &variable, float maximum, float minimum)
{
  if(variable > maximum)
  {
    variable = maximum;
  }
  else if(variable < minimum)
  {
    variable = minimum;
  }
};



