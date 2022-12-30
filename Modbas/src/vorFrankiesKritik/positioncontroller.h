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
    uVpk = 0.0F; //initialize output with zero
    uVpk_1 = 0.0F ; // initialize delayed output with zero
  }
  /**
  * @brief calculates the reference signals
  * @param[in] xs end of maneuver [m]
  * @param[in] vs max speed of maneuver [m/s]
  * @param[in] x0 start position of maneuver [m]
  * @param[in] t time [s]
  * @param[out] wp reference signal for position controller
  * @param[out] uVp1 input signal for feedforward controller 
  */
  void referenceSignal(const float xs, const float vs, const float x0, const float t, float& wp, float& uVp1)
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
     // Array that stores coefficients for wp
     std::array <float, 6> c = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
     c.at(5) =  65536.0F * vs * vs * vs * vs * vs
                         / xs / xs / xs / xs / 253125.0F;
     c.at(4) =  -4096.0F * vs * vs * vs * vs
                         / xs / xs / xs / 3375.0F;
     c.at(3) =   1024.0F * vs * vs * vs
                         / xs / xs / 675.0F;
     c.at(2) = 0.0F;
     c.at(1) = 0.0F;
     c.at(0) = x0;
     // calculation of reference signal for position controller
     wp = c.at(5) * t * t * t * t * t
         + c.at(4) * t * t * t * t
         + c.at(3) * t * t * t
         + c.at(2) * t * t
         + c.at(1) * t
         + c.at(0);

     // Array that stores coefficients for uVp1
     std::array <float, 6> cff = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
     cff.at(5) = c.at(5);
     cff.at(4) = c.at(4) + 5.0F * p->Ti * c.at(5) * ((1 / p->k / p->kr) + 1);
     cff.at(3) = c.at(3) + 4.0F * p->Ti * c.at(4) * ((1 / p->k / p->kr) + 1) + 20.0F * p->Ti * c.at(5) * (p->T + p->Tt) / p->k / p->kr;
     cff.at(2) = c.at(2) + 3.0F * p->Ti * c.at(3) * ((1 / p->k / p->kr) + 1) + 12.0F * p->Ti * c.at(4) * (p->T + p->Tt) / p->k / p->kr
         + 60.0f * p->T * p-> Ti * p->Tt * c.at(5) / p->k / p->kr;
     cff.at(1) = c.at(1) + 2.0F * p->Ti * c.at(2) * ((1 / p->k / p->kr) + 1) + 6.0F * p->Ti * c.at(3) * (p->T + p->Tt) / p->k / p->kr
         + 24.0f * p->T * p-> Ti * p->Tt * c.at(4) / p->k / p->kr;
     cff.at(0) = c.at(0) + p->Ti * c.at(1) * ((1 / p->k / p->kr) + 1) + 2.0F * p->Ti * c.at(2) * (p->T + p->Tt) / p->k / p->kr
         + 6.0f * p->T * p-> Ti * p->Tt * c.at(3) / p->k / p->kr;
     // calculation of reference signal for feedforward controller
     uVp1 = cff.at(5) * t * t * t * t * t
         + cff.at(4) * t * t * t * t
         + cff.at(3) * t * t * t
         + cff.at(2) * t * t
         + cff.at(1) * t
         + cff.at(0);
   }
  }

  /**
  * @brief feedforward controller
  * @param[in] uVp1k time discrete value of input signal
  * @param[in] uVp1k_1 last value of uVp1k
  * @param[out] uVp output signal from feedforward controller 
  */
  void feedforwardCtrl(const float uVp1k, float& uVp1k_1, float& uVp)
  {
    uVpk = (2.0F * uVp1k - 2.0F * uVp1k_1 - (p->Tak - 2.0F * p->Ti) * uVpk_1)/(p->Tak + 2.0F * p->Ti);
    uVp1k_1 = uVp1k; // save last value
    uVpk_1= uVpk; // save last value
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

private:
  float uVpk; // time discrete value of uVp
  float uVpk_1; // last value of uVpk

  CarParameters* p=CarParameters::p();//Access to Car Parameters
};

#endif // POSITIONCONTROLLER_H
