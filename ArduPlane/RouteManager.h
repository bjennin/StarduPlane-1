#ifndef ROUTE_MANAGER_H
#define ROUTE_MANAGER_H

#include "Route.h"

// Route Manager Class Definition
class RouteManager
{
  protected:
    // Route Number
    uint32_t routeNumber;
    static const uint32_t TURN_15DEG = 1;
    static const uint32_t TURN_45DEG = 2;
    
    // Route Information
    Route route;
  
  public:
    RouteManager();                            // Constructor
    void Initialize(uint32_t routeNumber);     // Initialization Routine
    float GetHeadingCommand();                 // Get heading command from specified route
    ~RouteManager();                           // Destructor
};
#endif /* ROUTE_MANAGER_H */
