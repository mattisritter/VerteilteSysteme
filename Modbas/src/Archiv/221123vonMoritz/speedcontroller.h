#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include <ros/ros.h>
#include "CarParameters.h"

#pragma once
// include STL array type
#include <array>
// include math
//#include <cmath>

class SpeedController{
public:
  using InputsType = std::array<float, 2>; /**< type definition for model input
  (array of floats with 3 elements) */
  //using StatesType = std::array<float, 3>; /**< type definition for model states */
  using OutputsType = std::array<float, 1>; /**< type definition for model output */
  /**
  * @brief Only constructor
  */
  SpeedController() // copy parameters to member variables
  {
    //x.fill(0.0F); // initialize state vector with zeros
    ek.fill(0.0F); // initialize input vector with zeros
    ukDelayed = 0.0F ;// initialize output Delay with zero

  }

  /**
  * @brief initializes model by initial conditions (IC)
  * @param[in] x0 initial model state vector
  */
  void init(/*const StatesType& x0*/)
  {
    t = 0.0F; // initialize simulation time to zero
    //x0; // copy initial vector to state vector
  }
  /**
  * @brief execute one single integration step
  * @param[in] u input vector
  * @param[out] y output vector
  * @param[in] dt sample time
  */
  void step(const InputsType& ek, OutputsType& uk, const float dt)
  {
    /*auskommentiert um deque zu verwenden: this->u = u;*/ // copy input vector to member variable u
    // one step of Runge Kutta
    // *this is this object, Runge Kutta calls this object as a functor
    // which means that the operator() is called at every
    // (major and minor) integration step
    const float ek = ek.at(1)/*wk*/ - ek.at(0)/*yk*/; //Control deviation
    const float upk = p->kr * ek;                     //Time discrete p-part
    const float uik = ukDelayed + p->kr * p->Tak * ek / p->Ti;//Time discrete i-part
    float ukStorage = 0.0F; //Cache for uk
    if (hold == 0)
    {
      ukStorage = upk + uik;
    }
    else
    {
      ukStorage = upk;
    }

    //Limit output signal and start clamping anti windup
    if (ukStorage >= 1)       /*limit control signal to 1*/
    {
      uk = 1;
      hold = 1;
    }
    else if (ukStorage <= -1) /*limit controll signal to -1*/
    {
      uk= -1;
      hold = 1;
    }
    else                     /*Pass controll signal*/
    {
      uk = ukStorage;
      hold = 0;
    }

    ukDelayed = uk;
    t += dt; // increase simulation time by sample time
  }
  /**
  * @brief operator () makes this class to a functor.
  * This operator is called by Runge Kutta internally.
  * @param[in] x the current state vector
  * @param[out] xd the current differential of the state vector
  * @param[in] t the current simulation time
  */
//  void operator()(const StatesType& x, StatesType& xd, float t)
//  {
//    //xd...

//  }
private:
  boost::numeric::odeint::runge_kutta4<StatesType> solver; /**< the Runge Kutta solver
  parameterized with states vector type */
  InputsType ek; /**< input signal */
  //StatesType ukDelayed; /**< state signal */
  float t = 0.0F; /**< simulation time */
  float ukDelayed;/**< Delay output for one step */
  bool hold = 0;


  CarParameters* p=CarParameters::p();//Access to Car Parameters

};

#endif // SPEEDCONTROLLER_H
