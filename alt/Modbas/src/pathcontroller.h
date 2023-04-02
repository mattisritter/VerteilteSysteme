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
#include <cmath>

class PathController{
public:
  /**
  * @brief Only constructor
  */
  PathController() // copy parameters to member variables
  {
  }

  /**
   * @brief initializes spline
   * @param[in] breaks List of arc lengths of waypoints [ m ]
   * @param[in] s List of x- and y-coordinates of waypoints [ m ]
   * @param[in] segmentIds Id of segment at every waypoint
   * @param[out] spline initialized spline
   */
  void init(std::vector<float>& breaks, std::array<std::vector<float>, 2>& s, std::vector<uint32_t>& segmentIDs, modbas::Spline& spline)
  {
    spline = modbas::Spline(breaks, s.at(0), s.at(1), segmentIDs);
  }

  /**
   * @brief calcualtion and limitation of control signal
   * @param[in] spline
   * @param[in] s actual cartesian center position [m]
   * @param[in] psi yaw angle [rad]
   * @param[in] vs max speed [m/s]
   * @param[out] delta control signal for steering
   */
  void step(modbas::Spline& spline, boost::array<float, 2>& s, const float psi, const float vs, float& delta)
  {
    // definition of variables
    // outputs of reference signal
    std::array<float, 2> ys; // interpolated spline coordinaties at xs [m, m]T
    float psis = 0.0F; // reference yaw angle at xs [rad]
    float kappah = 0.0F; // reference curavture at xh [1/m]
    // output of path controller
    float deltae = 0.0F; // steering deviation [ ]
    // output of feedforward control
    float deltas = 0.0F; //manipulation signal [rad]

    // calling private functions
    referenceSignal(s, vs, spline, ys, psis, kappah);
    pathCtrl(ys, psis, s, psi, vs, deltae);
    feedforwardCtrl(kappah, deltas);
    delta = deltas + deltae;
    delta = delta / p->deltaMax; // normalize delta
    if (delta >= 1.0F)
    {
      delta = 1.0F; // limit control signal to 1
    }
    else if (delta <= -1.0F)
    {
      delta = -1.0F; // limit control signal to -1
    }
  }

private:

  /**
   * @brief calculates reference signals
   * @param[in] sActual actual cartesian center position [m]
   * @param[in] vs max speed [m/s]
   * @param[in] spline
   * @param[out] ys interpolated spline coordinaties at xs [m, m]T
   * @param[out] psis reference yaw angle at xs [rad]
   */
  void referenceSignal(boost::array<float, 2>& sActual, const float vs, modbas::Spline& spline, std::array<float, 2>& ys,
                       float& psis, float& kappah)
  {
    float dist = 0.0F; // distance to spile [m]
    // nearest point
    float xs; // nearest arcposition on spline [m]
    std::array<float, 2> yds { { 0.0F, 0.0F } }; // interpolated tangential vector
    std::array<float, 2> ydds { { 0.0F, 0.0F } }; // interpolated second derivative
    // lookahed point
    float xh = 0.0F; // lookahead arcposition on spline [m]
    std::array<float, 2> yh; // interpolated spline coordinaties at xh [m, m]^T
    std::array<float, 2> ydh; // interpolated tangential vector
    std::array<float, 2> yddh; // interpolated second derivative

    std::array<float, 2> s; //convert boost::array to std::array
    s.at(0)= sActual.at(0);
    s.at(1)= sActual.at(1);
    int k = spline.getNearest(s, xs, dist);

    spline.interpolate(xs, ys, yds, ydds, k); // interpolate spline at nearest point
    xh = xs + vs * p->Tt; // calculate future arcposition to compensate deadtime
    spline.interpolate(xh, yh, ydh, yddh, -1); // interpolates spline at lookahead point

    psis = std::atan2(yds.at(1), yds.at(0)); // calculate reference yaw angle (9.2)
    float sign = 0.0F; // sign of curvature [ ]
    if (ydh.at(0) * yddh.at(1) - yddh.at(0) * ydh.at(1) >= 0.0F)
    {
      sign = 1.0F;
    }
    else
    {
      sign = -1.0F;
    }
    kappah = sign * std::sqrt(yddh.at(0) * yddh.at(0) + yddh.at(1) * yddh.at(1)); // calculate future curvature (9.5)
  }

  /**
   * @brief path controller
   * @param[in] ys interpolated spline coordinaties at xs [m, m]T
   * @param[in] psis reference yaw angle at xs [rad]
   * @param[in] s actual cartesian center position [m]
   * @param[in] psi actual yaw angle [rad]
   * @param[in] vs max speed [m/s]
   * @param[out] deltae steering deviation [ ]
   */
  void pathCtrl(const std::array<float, 2> ys, const float psis, boost::array<float, 2>& s, const float psi, const float vs, float& deltae)
  {
    // lateral control deviation [m]
    const float sc2e = (s.at(1)-ys.at(1))*std::cos(psis) - (s.at(0) - ys.at(0)) * std::sin(psis); // calculate lateral control deviation (9.6)
    // yaw angle deviation [rad]
    const float psie = modbas::Utils::normalizeRad(psi - psis); // calculate yaw deviation (9.8)
    float vE = 0.2F;
    float vsc = vs; // reference speed for calculation of control deviation
    if (vs >= 0.0F && vs <= vE)
    {
      vsc = vE;
    }
    else if (vs < 0.0F && vs >= -vE)
    {
      vsc = -vE;
    }
    deltae = -sc2e * p->l / (p->pathTw * p->pathTw * vsc * vsc) - 2.0F * psie * p->l / (p->pathTw * vsc); // calcualte control deviation
  }

  /**
   * @brief feedforward controller
   * @param[in] kappah reference curavture at xh [1/m]
   * @param[out] deltas manipulation signal [rad]
   */
  void feedforwardCtrl(const float kappah, float& deltas)
  {
    deltas = std::atan(p->l * kappah); // calculate manipulation signal
  }

  const CarParameters* const p=CarParameters::p(); // Access to Car Parameters
};
#endif // PATHCONTROLLER_H
