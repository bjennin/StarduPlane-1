#ifndef ROUTE_H
#define ROUTE_H

// Route Class Definition
class Route
{  
  protected:
    // Waypoint Information
    const uint32_t Ndim = 2;
    uint32_t Nwp = 0;
    uint32_t iwp = 0;
    float **waypoints;
    float *Hwp;
  
  public:
    Route(void);                                                 // Constructor
    void Initialize(uint32_t Nwp);                               // Initialization Routine
    void AddWaypoint(uint32_t i, float x, float y);              // Add Waypoint
    void ComputeWaypointHeadings(void);                          // Compute waypoint headings
    float ComputeHeadingCommand(uint32_t &route_complete_flag);  // Compute heading command
    void ClearWaypointData(void);                                // Clear all waypoints in route
    ~Route();                                                    // Destructor
};
#endif /* ROUTE_H */
