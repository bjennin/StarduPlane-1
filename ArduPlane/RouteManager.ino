#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "RouteManager.h"

// Constructor for the RouteManager class
RouteManager::RouteManager() {}

// Initialize the RouteManager
void RouteManager::Initialize(uint32_t routeNumber) 
{
  
  // Create route based on specified route number
  this->routeNumber = routeNumber;

  if (routeNumber == TURN_15DEG) {
    // Initialize route
    uint32_t Nwp = 3;        // Number of waypoints
    
    mRoute.Initialize(Nwp);
    
    // Add waypoints (Waypoint number, x position, y position)
    mRoute.AddWaypoint(0,  150.0, -100.0);
    mRoute.AddWaypoint(1,    0.0, -100.0);
    mRoute.AddWaypoint(2, -150.0, -100.0 + 150.0*tan(15*PI/180));
    
    // Compute waypoint headings for route
    mRoute.ComputeWaypointHeadings();
    
  }
  else if (routeNumber == TURN_45DEG) 
  {
    // Initialize route
    uint32_t Nwp = 3;        // Number of waypoints
    
    mRoute.Initialize(Nwp);
    
    // Add waypoints (Waypoint number, x position, y position)
    mRoute.AddWaypoint(0, 100.0, -150.0);
    mRoute.AddWaypoint(1, -50.0,    0.0);
    mRoute.AddWaypoint(2, -50.0,  150.0);
    
    // Compute waypoint headings for route
    mRoute.ComputeWaypointHeadings();
    
  }

}

// Get heading command from specified route
float RouteManager::GetHeadingCommand() {
  // Compute heading
  uint32_t route_complete_flag = 0;
  float headingCommand = mRoute.ComputeHeadingCommand(route_complete_flag);
  
  // Reinitialize route if route is complete
  if (route_complete_flag == 1) {
    Initialize(routeNumber);
  }
  return headingCommand;
}

// Destructor for the RouteManager class
RouteManager::~RouteManager() {}
