#ifndef POSITIONCONTROLLER_H
#define POSITIONCONTROLLER_H

#include <ros/ros.h>
#include "CarParameters.h"

#pragma once
// include STL array type
#include <array>

class PositionController{
public:
  /**
  * @brief Only constructor
  */
  PositionController()
  {
  }

  /**
  * @brief initialize position controller
  * @param[in] x actual arcposition [m]
  * @param[in] xs end of maneuver [m]
  * @param[in] vs max speed of maneuver [m/s]
  * @param[in] t time [s]
  */
  void init(const float x, const float xs, const float vs)
  {
    t = 0.0F; // reset time
    x0 = x; // set start arclength to current arclength

    uVpkm1 = 0.0F;
    uVpk = 0.0F;
    uVp1km1 = x; // set the old value of uVp1 to current arclength

    // caculating coefficients for uVp1
    c.at(5) = 65536.0F * vs * vs * vs * vs * vs / (xs * xs * xs * xs * 253125.0F);
    c.at(4) = -4096.0F * vs * vs * vs * vs / (xs * xs * xs * 3375.0F);
    c.at(3) =  1024.0F * vs * vs * vs / (xs * xs * 675.0F);
    c.at(2) = 0.0F;
    c.at(1) = 0.0F;
    c.at(0) = x0;

    // calculation coefficients for uVp1
    // precalculation
    const float a1 = p->Ti * (1.0F / (p->k * p->kr) + 1.0F);
    const float a2 = p->Ti * (p->T + p->Tt) / (p->k * p->kr);
    const float a3 = p->Ti * p->T * p->Tt / (p->k * p->kr);
    cff.at(5) = c.at(5);
    cff.at(4) = c.at(4) + 5.0F * c.at(5) * a1;
    cff.at(3) = c.at(3) + 4.0F * c.at(4) * a1 + 20.0F * c.at(5) * a2;
    cff.at(2) = c.at(2) + 3.0F * c.at(3) * a1 + 12.0F * c.at(4) * a2 + 60.0F * a3 * c.at(5);
    cff.at(1) = c.at(1) + 2.0F * c.at(2) * a1 +  6.0F * c.at(3) * a2 + 24.0F * a3 * c.at(4);
    cff.at(0) = c.at(0) + 1.0F * c.at(1) * a1 +  2.0F * c.at(2) * a2 +  6.0F * a3 * c.at(3);
  }

  /**
  * @brief step of position controller
  * @param[in] xs end of maneuver [m]
  * @param[in] vs max speed of maneuver [m/s]
  * @param[in] x actual arcposition [m]
  * @param[out] up control variable
  */
  void step(const float xs, const float vs, const float x, float& up)
  {
    float uVp = 0.0F;
    float uRp = 0.0F;
    referenceSignal(xs, vs, t, wp, uVp1); // calulate Reference Signals
    feedforwardCtrl(uVp1, uVp); // online feed forward control
    float ep = wp - x;
    positionCtrl(ep, uRp);
    up = uVp + uRp; // calculate control signal for position control

    t += p->Tak;
  }

  // for debugging
  float getwp(){return wp;}
  float getuvp1(){return uVp1;}

private:
  /**
  * @brief calculates the reference signals
  * @param[in] xs end of maneuver [m]
  * @param[in] vs max speed of maneuver [m/s]
  * @param[in] t time [s]
  * @param[out] wp reference signal for position controller
  * @param[out] uVp1 input signal for feedforward controller
  */
  void referenceSignal(const float xs, const float vs, const float t, float& wp, float& uVp1)
  {
   const float te = 15.0F * xs /  8.0F / vs; // Endtime of parking maneuver
   // freezes signals to end position of parking maneuver if t > te
   if (t > te)
   {
     wp = x0 + xs;
     uVp1 = x0 + xs;
   }
   else
   {
     // calculation of reference signal for position controller
     wp = ((((c.at(5) * t + c.at(4)) * t + c.at(3)) * t + c.at(2)) * t + c.at(1)) * t + c.at(0);
     // calculation of reference signal for feedforward controller
     uVp1 = ((((cff.at(5) * t + cff.at(4)) * t + cff.at(3)) * t + cff.at(2)) * t + cff.at(1)) * t + cff.at(0);
   }
  }

  /**
  * @brief feedforward controller
  * @param[in] uVp1k time discrete value of input signal
  * @param[out] uVp output signal from feedforward controller
  */
  void feedforwardCtrl(const float uVp1k, float& uVp)
  {
    uVpk = (2.0F * uVp1k - 2.0F * uVp1km1 - (p->Tak - 2.0F * p->Ti) * uVpkm1) / (p->Tak + 2.0F * p->Ti);
    uVp1km1 = uVp1k; // save last value
    uVpkm1= uVpk; // save last value
    uVp = uVpk; // assign time discrete value to the output of the feedforward controller
  }

  /**
  * @brief execute one single step
  * @param[in] ep control deviation [m]
  * @param[out] control signal [m/s]
  */
  void positionCtrl(const float ep, float& uRp)
  {
    uRp = p->kp * ep; //Calculate uRp
  }

  // member variables
  std::array<float, 6> cff = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 6> c = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  float t = 0.0F;
  float x0 = 0.0F;
  float uVpk = 0.0F;
  float uVpkm1 = 0.0F;
  float uVp1km1 = 0.0F;

  // for debugging only needed in step
  float wp = 0.0F;
  float uVp1 = 0.0F;

  CarParameters* p=CarParameters::p(); // access to Car Parameters
};

#endif // POSITIONCONTROLLER_H

