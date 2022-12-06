#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include <ros/ros.h>
#include "CarParameters.h"

#pragma once
// include STL array type
#include <array>

class SpeedController{
public:
  using InputsType = std::array<float, 2>; /**< type definition for model input
  (array of floats with 2 elements) */
  using OutputsType = std::array<float, 1>; /**< type definition for model output */
  /**
  * @brief Only constructor
  */
  SpeedController() // copy parameters to member variables
  {
    e.fill(0.0F); // initialize input vector with zeros
    uikDelayed = 0.0F ;// initialize output Delay with zero
  }

  /**
  * @brief initializes model by initial conditions (IC)
  * @param[in] x0 initial model state vector
  */
  void init()
  {
  }
  /**
  * @brief execute one single integration step
  * @param[in] u input vector
  * @param[out] y output vector
  * @param[in] dt sample time
  */
  void step(const InputsType& e, OutputsType& uk)
  {
    const float ek = e.at(1) - e.at(0); //Control deviation: ek = wk - yk
    const float upk = p->kr * ek;                 //Time discrete p-part
    float ukStorage = 0.0F; //Cache for uk

    //Clamping-Anti-Windup
    if (hold == 0)
    {
      const float uik = uikDelayed + p->kr * p->Tak * ek / p->Ti; //Time discrete i-part
      ukStorage = upk + uik;
      uikDelayed = uik;
    }
    else
    {
      //const float uik = uikDelayed + p->kr * p->Tak * ek / p->Ti; //Time discrete i-part
      //As ek=0 -> equation result equals uikDelayed -> Calculation is performed with uikDelayed
      ukStorage = upk + uikDelayed;
    }

    //Limit output signal and start clamping anti windup
    if (ukStorage >= 1.0F)       /*limit control signal to 1*/
    {
      uk.at(0) = 1.0F;
      hold = 1;
    }
    else if (ukStorage <= -1.0F) /*limit control signal to -1*/
    {
      uk.at(0) = -1.0F;
      hold = 1;
    }
    else                         /*Pass control signal*/
    {
      uk.at(0) = ukStorage;
      hold = 0;
    }
  }
  /**
  * @brief operator () makes this class to a functor.
  * This operator is called by Runge Kutta internally.
  * @param[in] x the current state vector
  * @param[out] xd the current differential of the state vector
  * @param[in] t the current simulation time
  */

private:
  //boost::numeric::odeint::runge_kutta4<StatesType> solver; /**< the Runge Kutta solver
  //parameterized with states vector type */
  InputsType e; /**< input signal */
  float uikDelayed;/**< Delay output for one step */
  bool hold = 0; //Flag, 1 if Anti-Windup is activated
  CarParameters* p=CarParameters::p();//Access to Car Parameters
};

#endif // SPEEDCONTROLLER_H
