#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

#include <ros/ros.h>
#include "CarParameters.h"
#include "Spline.h"

#pragma once
// include STL array type
#include <array>
// include STL vector type
#include <vector>

class PathController{
public:

  using PathInSType = std::array<std::vector<float>, 3>; /**< type definition for model input(array of vectors with 5 elements) */
  using PathInAType = std::array<float, 4>; /**< type definition for model input(array of floats with 3 elements) */
  using PathOutType = std::array<float, 2>; /**< type definition for model output */
  /**
  * @brief Only constructor
  */
  PathController() // copy parameters to member variables
  {
    //e.fill(0.0F); // initialize input vector with zeros
    //uVpk = 0.0F ;// initialize output with zero
    xs = 0.0F;
    pathInA.at(1) = 0.0F;
    pathInA.at(2) = 0.0F;

    ys = {0.0F, 0.0F};
    yds = {0.0F, 0.0F};
    ydds = {0.0F, 0.0F};
    yh = {0.0F, 0.0F};
    ydh = {0.0F, 0.0F};
    yddh = {0.0F, 0.0F};
    dist = 0.0F;
  }


  void init(const PathInSType& pathInS, const PathInSType& pathInA)
  {
    /**
    @brief Spline constructor interpolates waypoints
    @param[in] breaks List of arc lengths of waypoints [ m ]
    @param[in] vals0 List of x-coordinates of waypoints [ m ]
    @param[in] vals1 List of y-coordinates of waypoints [ m ]
    @param[in] segmentIds Id of segment at every waypoint
    */
    /*explicit*/ modbas::Spline spline(pathInS.at(0),
                    pathInS.at(1),
                    pathInS.at(2),
                    segmentIds); /*noexcept*/

     /**
     @brief getNearest returns nearest point on spline
     (point which has minimal distance to y)
     @param[in] y Coordinates of point next to spline (e.g., car position) [ m ]
     @param[out] x Arc length of nearest point on spline [ m ]
     @param[out] dist [ m ] Approximate distance to spline. Do not use for control functions
     @return Waypoint index of corresponding spline interval
     */
    std::array<float, 2> ys ;
    ys.at(0)= s.at(0);//{ pathInA.at(1), pathInA.at(2) };
    ys.at(1)= s.at(1);
    int k = spline.getNearest(ys , xs, dist);

    /**
    @brief interpolate interpolates on the spline
    @param[in] x The arc length of the point to be interpolated [ m ]
    @param[out] y The coordinates of the interpolated point [ m ]
    @param[out] yd The first derivative of the coordinates (tangential vector) [ 1 ]
    @param[out] ydd The second derivative of the coordinates (normal vector) [ 1/m ]
    @param[in] pieceIdx Optional waypoint index of interval on spline
    to speed up interpolation (default -1: search for interval on spline)
    */
    spline.interpolate(xs, ys, yds, ydds, /*const int pieceIdx = */-1);
    xh = xs + vmax * p->deltaTt; //xhat = xshould + vmax * Tt;
    spline.interpolate(xh, yh, ydh, yddh, /*const int pieceIdx = */-1);

  }

  void step()
  {

  }



private:
  //Parameters
  float xs;
  float dist;
  std::array<float, 2> ys;
  std::array<float, 2> yds;
  std::array<float, 2> ydds;
  float xh;
  std::array<float, 2> yh;
  std::array<float, 2> ydh;
  std::array<float, 2> yddh;

  //Inputs
  boost::array<float, 2> s;
  float vmax;
  std::vector<uint32_t> segmentIds;
  PathInSType pathInS;
  PathInAType pathInA;

  CarParameters* p=CarParameters::p();//Access to Car Parameters
};
#endif // PATHCONTROLLER_H
