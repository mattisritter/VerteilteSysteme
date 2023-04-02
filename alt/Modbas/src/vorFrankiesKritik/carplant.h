#ifndef CARPLANT_H
#define CARPLANT_H

#include <ros/ros.h>
#include "CarParameters.h"

#pragma once
// include STL array type
#include <array>
// include boost odeint
#include <boost/numeric/odeint.hpp>
// include math
#include <cmath>
// include deque
#include <deque>

/**
* @brief The CarPlant class
*/

class CarPlant{
public:
  using InputsType = std::array<float, 3>; /**< type definition for model input */
  using StatesType = std::array<float, 7>; /**< type definition for model states */
  using OutputsType = std::array<float, 6>; /**< type definition for model output */
  /**
* @brief Only constructor
*/
  CarPlant() // copy parameters to member variables
  {
    x = p->x0; // initialize state vector with initial values
    u.fill(0.0F); // initialize input vector with zeros

    // prefill deques with zeros
    unDeque.resize( 3, 0.0F );
    deltanDeque.resize( 3, 0.0F );
  }

  /**
* @brief initializes model by initial conditions (IC)
* @param[in] x0 initial model state vector
*/
//  void init(/*const StatesType& x0*/)
//  {
//    t = 0.0F; // initialize simulation time to zero
//    x = p->x0; // copy initial vector to state vector
//  }
  /**
* @brief execute one single integration step
* @param[in] u input vector
* @param[out] y output vector
* @param[in] dt sample time
*/
  void step(const InputsType& u, OutputsType& y, const float dt)
  {
    // time delay of pedals and steering
    unDeque.push_front(u.at(1));
    deltanDeque.push_front(u.at(2));
    unDeque.pop_back();
    deltanDeque.pop_back();
    un_Tt = unDeque.at(2);
    deltan_Tt = deltanDeque.at(2);

    // calculating the side slip angle for diffrent models
    float beta = 0.0F;
    if (std::abs(x.at(0)) > p->speedMin)
    {
      // dynamics model for fast driving
      beta = std::atan2(x.at(5), x.at(0)); // (5.23)
    }
    else
    {
      // dynamics model for slow driving
      beta = std::atan(p->lr / p->l * std::tan(p->deltaMax * deltan_Tt)); // (5.14)
    }
    // solve the differential equations with runge-kutta
    solver.do_step(*this, x, t, dt);

    // calculation of output signal
    y.at(0) = x.at(1) - p->lr * std::cos(x.at(3));
    y.at(1) = x.at(2) - p->lr * std::sin(x.at(3));
    y.at(2) = x.at(3);
    y.at(3) = beta;
    y.at(4) = x.at(0);
    y.at(5) = x.at(6);

    t += dt; // increase simulation time by sample time
  }
  /**
* @brief operator () makes this class to a functor.
* This operator is called by Runge Kutta internally.
* @param[in] x the current state vector
* @param[out] xd the current differential of the state vector
* @param[in] t the current simulation time
*/
  void operator()(const StatesType& x, StatesType& xd, float t)
  {
    const float delta = p->deltaMax * deltan_Tt; // steering angle (5.18)
    if (std::abs(x.at(0)) > p->speedMin)
    {
     // dynamics model for fast driving (Book page 172f)
     // slip angle of front (5.19) and rear wheel (5.20)
     float alphaf = 0.0F;
     float alphar = 0.0F;
     if (x.at(0) > 0.0F)
     {
       alphaf = -std::atan((x.at(5) + p->lf * x.at(4)) / x.at(0)) + delta;
       alphar = -std::atan((x.at(5) - p->lr * x.at(4)) / x.at(0));
     }
     else
     {
       alphaf = std::atan((x.at(5) + p->lf * x.at(4)) / x.at(0)) - delta;
       alphar = std::atan((x.at(5) - p->lr * x.at(4)) / x.at(0));
     }

     // orthogonal front wheel force (5.21)
     const float Ff = p->Df * std::sin(p->Cf * std::atan(p->Bf * (1.0F - p->Ef) * alphaf - p->Ef * std::atan(p->Bf * alphaf)));
     // orthogonal rear wheel force (5.22)
     const float Fr = p->Dr * std::sin(p->Cr * std::atan(p->Br * (1.0F - p->Er) * alphar - p->Er * std::atan(p->Br * alphar)));
     // differential equations, ordered by states
     // (5.26)
     xd.at(0) = ( -Ff * std::sin(delta) + p->m * x.at(5) * x.at(4)) / p->m - x.at(0) / p->T + p->k * un_Tt / p->T;
     // (5.24)
     xd.at(1) = x.at(0) * std::cos(x.at(3)) - x.at(5) * std::sin(x.at(3));
     // (5.25)
     xd.at(2) = x.at(0) * std::sin(x.at(3)) + x.at(5) * std::cos(x.at(3));
     // (5.28)
     xd.at(3) = x.at(4);
     // (5.29)
     xd.at(4) = (Ff * p->lf * std::cos(delta) - Fr * p->lr) / p->J;
     // (5.27)
     xd.at(5) = (Ff * std::cos(delta) + Fr - p->m * x.at(0) * x.at(4)) / p->m;
     // (5.30)
     xd.at(6) = x.at(0);
    }
    else
    {
      // dynamics model for slow driving
      const float beta = std::atan(p->lr * std::tan(delta) / p->l); // calculate side slip angle (5.14)

      // state space model (5.15)
      xd.at(0)= -x.at(0) / p->T + p->k * un_Tt / p->T;
      xd.at(1)= x.at(0) / std::cos(beta) * std::cos(x.at(3) + beta);
      xd.at(2)= x.at(0) / std::cos(beta) * std::sin(x.at(3) + beta);
      xd.at(3)= x.at(0) * std::tan(delta) / p->l;
      xd.at(4)= 0.0F; // Not used in this model
      xd.at(5)= 0.0F; // Not used in this model
      xd.at(6)= x.at(0);
    }
  }

private:
  boost::numeric::odeint::runge_kutta4<StatesType> solver; /**< the Runge Kutta solver parameterized with states vector type */
  InputsType u; // system input (cmd, pedals, steering)
  StatesType x; // system state (vc1, s1, s2, psi, omega, vc2, x)
  float t = 0.0F; // simulation time [s]

  CarParameters* p=CarParameters::p(); // access to Car Parameters

  // deques to impelent time delay
  std::deque <float> unDeque;
  std::deque <float> deltanDeque;
  // time delayed input signals
  float un_Tt = 0.0F; // delayed pedals signal: un(t-Tt)
  float deltan_Tt = 0.0F; // delayed steering signal: deltan(t-Tt)
};

#endif // CARPLANT_H
