#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "RouteManager.h"

// Constructor for the Route class
Route::Route() {}

// Initialize the RouteManager
void Route::Initialize(uint32_t Nwp) {
  // Clear waypoints
  for (uint32_t i=0; i<this->Nwp; i++) {
    delete[] waypoints[i]; waypoints[i] = NULL;
  }
  
  // Initialize waypoints
  this->Nwp = Nwp;
  for (uint32_t i=0; i<Nwp; i++) {
    waypoints[i] = new float[Ndim];
  }
  
  // Set default route
  for (uint32_t i=0; i<Nwp; i++) {
    for (uint32_t j=0; i<Ndim; i++) {
      waypoints[i][j] = 0.0;
    }
  }
  
  // Set waypoint iterator
  iwp = 0;
}

// Add Waypoint
void Route::AddWaypoint(uint32_t i, float x, float y) {
  waypoints[i][0] = x;
  waypoints[i][1] = y;
}

// Compute waypoint headings from specified waypoints
void Route::ComputeWaypointHeadings() {
  // Clear waypoint headings
  delete[] Hwp; Hwp = NULL;
  
  // Initialize and compute waypoint headings
  Hwp = new float[Nwp];
  float dx = waypoints[0][0] - X_position;
  float dy = waypoints[0][1] - Y_position;
  Hwp[0] = atan2(dy,dx);
  for (uint32_t i=1; i<Nwp; i++) {
    dx = waypoints[i][0] - waypoints[i-1][0];
    dy = waypoints[i][1] - waypoints[i-1][1];
    Hwp[i] = atan2(dy,dx);
  }
}

// Compute heading command based on current waypoint and position
float Route::ComputeHeadingCommand(uint32_t &route_complete_flag) {
  // Compute heading (UAV to waypoint)
  float dx = waypoints[iwp][0] - X_position;
  float dy = waypoints[iwp][1] - Y_position;
  float Huav = atan2(dy,dx);
  
  // Go to next waypoint if current waypoint is found
  float pos_error = sqrt(dx*dx + dy*dy);

  //hal.console->printf_P(PSTR("\n pos_error: %f \n"), pos_error);

  if (pos_error <= POSITION_ERROR) {
    iwp++;
  }
  
  // If all waypoints complete, set flag to alert RouteManager
  if (iwp == Nwp+1) {
    route_complete_flag = 1;
    return Hwp[Nwp-1];
  }
  
  // Compute heading (UAV to waypoint)
  dx = waypoints[iwp][0] - X_position;
  dy = waypoints[iwp][1] - Y_position;
  Huav = atan2(dy,dx);
  
  // Compute heading error (rad)
  float Herr = fabs(Huav - Hwp[iwp]);
  
  // Determine shortest angle and compute heading command
  float Hcom;
  if (Herr < (2*PI - Herr)) {
    Hcom = Hwp[iwp] + copysignf(1.0, Huav - Hwp[iwp])*ROUTE_P*Herr;
  }
  else {
    Herr = 2*PI - Herr;
    Hcom = Hwp[iwp] - copysignf(1.0, Huav - Hwp[iwp])*ROUTE_P*Herr;
  }
  
  // Check radian range of heading command
  if(Hcom > 2*PI) {
    Hcom -= 2*PI;
  }
  else if(Hcom < 0) {
    Hcom += 2*PI;
  }
  return Hcom;
}

// Destructor for the Route class
Route::~Route() {
  // Clear waypoints
  for (uint32_t i=0; i<Nwp; i++) {
    delete[] waypoints[i]; waypoints[i] = NULL;
  }
  
  // Clear waypoint headings
  delete[] Hwp; Hwp = NULL;
}
