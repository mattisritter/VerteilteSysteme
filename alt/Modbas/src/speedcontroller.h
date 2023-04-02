#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include <ros/ros.h>
#include "CarParameters.h"

#pragma once
// include STL array type
#include <array>

class SpeedController{
public:
  /**
  * @brief Only constructor
  */
  SpeedController()
  {
  }

  /**
  * @brief execute one single step
  * @param[in] wv reference speed [m/s]
  * @param[in] v speed [m/s]
  * @param[out] uv control variable
  */
  void step(const float wv, const float v, float& uv)
  {
    const float ev = wv - v;
    const float uvik = uvikm1 + p->kr * p->Tak * ev / p->Ti; // time discrete i-part
    const float uvpk = p->kr * ev; // time discrete p-part

    uv = uvpk + uvik;

    // Limit output signal and start clamping anti windup
    if (uv > 1.0F)       /*limit control signal to 1*/
    {
      uv = 1.0F;
    }
    else if (uv < -1.0F) /*limit control signal to -1*/
    {
      uv = -1.0F;

    }
    else                   /*Pass control signal*/
    {
      uvikm1 = uvik; // continue i-part only if not in windup
    }
    // set control signal to zero if reference signal is also zero
    if (wv == 0.0F)
    {
      uv = 0.0F;
      uvikm1 = 0.0F;
    }
  }

private:
  float uvikm1 = 0.0F; // last value of uvik
  
  const CarParameters* const p=CarParameters::p(); // Access to Car Parameters
};

#endif // SPEEDCONTROLLER_H
