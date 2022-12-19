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
    uvik_1 = 0.0F ; // initialize output Delay with zero
  }

  /**
  * @brief execute one single step
  * @param[in] ev control deviation
  * @param[out] uv control variable
  */
  void step(const float& ev, float& uv)
  {
    float uvk = 0.0F; // time discrete control signal
    const float uvpk = p->kr * ev; // time discrete p-part

    // Clamping-Anti-Windup
    if (hold == 0)
    {
      const float uvik = uvik_1 + p->kr * p->Tak * ev / p->Ti; // time discrete i-part
      uvk = uvpk + uvik;
      uvik_1 = uvik;
    }
    else
    {
      // As ek=0 -> equation result equals uvik_1 -> Calculation is performed with uvik_1
      uvk = uvpk + uvik_1;
    }

    // Limit output signal and start clamping anti windup
    if (uvk >= 1.0F)       /*limit control signal to 1*/
    {
      uv = 1.0F;
      hold = 1;
    }
    else if (uvk <= -1.0F) /*limit control signal to -1*/
    {
      uv = -1.0F;
      hold = 1;
    }
    else                   /*Pass control signal*/
    {
      uv = uvk;
      hold = 0;
    }
  }

private:
  float uvik_1; // last value of uvik
  bool hold = 0; // Flag, 1 if Anti-Windup is activated
  
  CarParameters* p=CarParameters::p(); // Access to Car Parameters
};

#endif // SPEEDCONTROLLER_H
