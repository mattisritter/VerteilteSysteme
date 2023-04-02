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
    float uvk = 0.0F; // time discrete control signal
    const float uvpk = p->kr * ev; // time discrete p-part

    // Clamping-Anti-Windup
    if (hold == false)
    {
      uvikm1 = uvikm1 + p->kr * p->Tak * ev / p->Ti; // time discrete i-part
    }
    uvk = uvpk + uvikm1;

    // Limit output signal and start clamping anti windup
    if (uvk >= 1.0F)       /*limit control signal to 1*/
    {
      uv = 1.0F;
      hold = true;
    }
    else if (uvk <= -1.0F) /*limit control signal to -1*/
    {
      uv = -1.0F;
      hold = true;
    }
    else                   /*Pass control signal*/
    {
      uv = uvk;
      hold = false;
    }
  }

private:
  float uvikm1 = 0.0F; // last value of uvik
  bool hold = false; // Flag, 1 if Anti-Windup is activated
  
  const CarParameters* const p=CarParameters::p(); // Access to Car Parameters
};

#endif // SPEEDCONTROLLER_H
