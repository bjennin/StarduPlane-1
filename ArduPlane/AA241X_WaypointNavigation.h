#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

/**** Waypoint Navigation ****/
#define Ndim 2                                 // Dimension of waypoints
#define Np 4                                   // Number of persons to find
static char persons_found[Np] = {0,0,0,0};     // Number of persons found
static uint32_t iwp = 0;                       // Waypoint iterator

// Waypoint data for first part of mission
#define Nwp 62
static float waypoints[Nwp][Ndim] = {
{ -44.6714,  144.0579},
{ -74.8749,  130.9273},
{-101.5082,  111.5539},
{-123.3015,   86.8616},
{-139.2157,   58.0275},
{-148.4919,   26.4266},
{-150.6878,   -6.4343},
{-145.6987,  -38.9884},
{-133.7625,  -69.6835},
{-115.4484,  -97.0561},
{ -91.6295, -119.8008},
{ -63.4416, -136.8333},
{ -32.2288, -147.3415},
{   0.5208, -150.8242},
{  33.2455, -147.1154},
{  64.3850, -136.3920},
{  92.4546, -119.1652},
{ 116.1158,  -96.2565},
{ 134.2405,  -68.7582},
{ 145.9644,  -37.9813},
{ 150.7286,   -5.3935},
{ 148.3058,   27.4514},
{ 138.8117,   58.9875},
{ 122.6988,   87.7110},
{ 100.7354,  112.2523},
{  73.9689,  131.4412},
{  43.6755,  144.3629},
{  11.2995,  150.4012},
{ -21.6152,  149.2682},
{ -44.6780,  117.7661},
{ -55.5168,   84.4782},
{ -79.6754,   62.2134},
{ -95.4183,   33.3772},
{-101.0824,    1.0155},
{ -96.0696,  -31.4535},
{ -80.9092,  -60.6002},
{ -57.2028,  -83.3458},
{ -27.4542,  -97.2880},
{   5.1943, -100.9540},
{  37.2942,  -93.9565},
{  65.4547,  -77.0348},
{  86.7016,  -51.9762},
{  98.7904,  -21.4275},
{ 100.4444,   11.3845},
{  91.4888,   42.9940},
{  72.8695,   70.0622},
{  46.5533,   89.7300},
{  15.3199,   99.9199},
{ -13.2753,   75.3850},
{ -24.9661,   45.6175},
{ -47.1497,   21.9356},
{ -50.9748,  -10.2873},
{ -34.9522,  -38.5046},
{  -5.3204,  -51.7296},
{  26.3829,  -44.8130},
{  47.8137,  -20.4477},
{  50.6275,   11.8791},
{  33.7288,   39.5806},
{   3.6973,   51.8709},
{ -16.6920,   26.4225},
{  -9.7828,    3.8258},
{   9.7828,   -3.8258}
};

/*
// Waypoint data for first 15 degree route
#define Nwp 3
static float waypoints[Nwp][Ndim] = {
{ 150.0, -100.0},
{   0.0, -100.0},
{-150.0, -59.8076}
};
*/

/*
// Waypoint data for first 45 degree route
#define Nwp 3
static float waypoints[Nwp][Ndim] = {
{ 100.0, -150.0},
{ -50.0,    0.0},
{ -50.0,  150.0}
};
*/

static float Hwp[Nwp];

// Wrap angle to interval [0,2*pi]
static float WrapAngle(float angle) {
  float twoPi = 2*PI;
  return angle - twoPi*(float)floor(angle/twoPi);
}

#endif /* WAYPOINT_NAVIGATION_H */

