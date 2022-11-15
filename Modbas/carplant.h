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
  using InputsType = std::array<float, 3>; /**< type definition for model input
(array of floats with 3 elements) */
  using StatesType = std::array<float, 7>; /**< type definition for model states */
  using OutputsType = std::array<float, 6>; /**< type definition for model output */
  /**
* @brief Only constructor
*/
  CarPlant() // copy parameters to member variables
  {
    x.fill(0.0F); // initialize state vector with zeros
    u.fill(0.0F); // initialize input vector with zeros
  }

  /**
* @brief initializes model by initial conditions (IC)
* @param[in] x0 initial model state vector
*/
  void init(const StatesType& x0)
  {
    t = 0.0F; // initialize simulation time to zero
    x = x0; // copy initial vector to state vector
  }
  /**
* @brief execute one single integration step
* @param[in] u input vector
* @param[out] y output vector
* @param[in] dt sample time
*/
  void step(const InputsType& u, OutputsType& y, const float dt)
  {
    /*auskommentiert um deque zu verwenden: this->u = u;*/ // copy input vector to member variable u
    // one step of Runge Kutta
    // *this is this object, Runge Kutta calls this object as a functor
    // which means that the operator() is called at every
    // (major and minor) integration step

    u1Deque.push_front(u.at(0));
    u2Deque.push_front(u.at(1));
    u3Deque.push_front(u.at(2));
    u1Deque.pop_back();
    u2Deque.pop_back();
    u3Deque.pop_back();

    uDelayed.at(0) = u1Deque.at(29);
    uDelayed.at(1) = u2Deque.at(29);
    uDelayed.at(2) = u3Deque.at(29);
    //this->uDelayed = uDelayed;

    float beta = 0;
    if(x.at(0) < 0.2/*m/s*/) /*calculating beta for different models*/
    {
      //Dynamics model for slow driving
      beta = atan(p->lr/p->l*tan(p->deltaMax*uDelayed.at(2)));
    }
    else
    {
      //Dynamics model for fast driving
      beta = atan2(x.at(5),x.at(0));//5.23
    }
    /*Todo: wie soll delay eingebracht werden?*/
    /*Todo: Darstellung von y vektor richtig, muss s vektor aufgestellt werden?*/
    solver.do_step(*this, x, t, dt);
    y.at(0) = x.at(1)-p->lr*cos(x.at(3));
    y.at(1) = x.at(2)-p->lr*cos(x.at(3));
    y.at(2) = x.at(3);
    y.at(3) = beta;
    y.at(4) = x.at(0);

//    /*if(x.at(0) < 0.2/*m/s*/) /*calculating y6 for different models*/
//    {
//      //Dynamics model for slow driving

//      y.at(5) = x.at(4);
//    }
//    else
//    {

      //Dy*/namics model for fast driving
     y.at(5) = x.at(6);
   // }

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
    if(x.at(0) < 0.2/*m/s*/)
    {
      //Dynamics model for slow driving
      float beta = atan((p->lr) / (p->l) * tan((p->deltaMax) * uDelayed.at(2))); //calculate beta
      //input delay
      /*todo totzeit, ungewiss ob richtig*///resize für deque!!!!!!!!!!!!!!!!!!!!!!!!!
      //5.15
      xd.at(0)=(-x.at(0)) / p->T + p->k / p->T * uDelayed.at(1);
      xd.at(1)= x.at(0) / cos(beta) * cos(x.at(3) + beta);
      xd.at(2)= x.at(0) / cos(beta) * sin(x.at(3) + beta);
      xd.at(3)= x.at(0) / (p->l) * tan(p->deltaMax * uDelayed.at(2));
      xd.at(4)= 0.0F; //Not used in this model
      xd.at(5)= 0.0F; //Not used in this model
      xd.at(6)= x.at(0);
     /* xd.at(4) = x.at(0);
      xd.at(5)= 0.0F; //Not used in this model
      xd.at(6)= 0.0F; //Not used in this model*/

    }
    else
    {
       //Dynamics model for fast driving (Book page 172f)
      float deltaT = p->deltaMax * uDelayed.at(2); //5.18
      //5.19
      float alphaf = 0;
      if(x.at(0) > 0)
      {
        alphaf =-atan((x.at(5) + p->lf * x.at(4)) / x.at(0) + deltaT);
      }
      else if(x.at(0) < 0)
      {
        alphaf = atan((x.at(5) + p->lf * x.at(4)) / x.at(0) - deltaT);
      }
      //5.20
      float alphar = 0;
      if(x.at(0) > 0)
      {
        alphar =-atan((x.at(5) - p->lr * x.at(4)) / x.at(0));
      }
      else if(x.at(0) < 0)
      {
        alphar = atan((x.at(5) - p->lr * x.at(4)) / x.at(0));
      }
      //5.21
      float Ff = p->Df * sin(p->Cf * atan(p->Bf *(1-p->Ef) * alphaf - p->Ef * atan(p->Bf * alphaf)));
      //5.22
      float Fr = p->Dr * sin(p->Cr * atan(p->Br * (1-p->Er) * alphar - p->Er * atan(p->Br * alphar)));
      //Differential equations, ordered by states numbers
      //5.26
      xd.at(0) = (-Ff * sin(deltaT) + p->m * x.at(5) * x.at(4)) / p->m - x.at(0) / p->T + p->k * uDelayed.at(1) / p->T;
      //5.24
      xd.at(1) = x.at(0) * cos(x.at(3)) - x.at(5) * sin(x.at(3));
      //5.25
      xd.at(2) = x.at(0) * sin(x.at(3)) + x.at(5) * cos(x.at(3));
      //5.28
      xd.at(3) = x.at(4);
      //5.29
      xd.at(4) = (Ff * p->lf * cos(deltaT) - Fr * p->lr) / p->J;
      //5.27
      xd.at(5) = (Ff * cos(deltaT) + Fr - p->m * x.at(0) * x.at(4)) / p->m;
      //5.30
      xd.at(6) = x.at(0);
    }

  }
private:
  boost::numeric::odeint::runge_kutta4<StatesType> solver; /**< the Runge Kutta solver
parameterized with states vector type */
  InputsType u; /**< input signal */
  StatesType x; /**< state signal */
  float t = 0.0F; /**< simulation time */

  CarParameters* p=CarParameters::p();//Access to Car Parameters

  /*Definition of time delay variables*/
  //Deques to impelent timedelay
  std::deque<float>u1Deque = {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; //Prefill with zeros, 30 times
  std::deque<float>u2Deque = {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};
  std::deque<float>u3Deque = {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};
  //Array to store deques
  std::array<float,3>uDelayed = {0.0F, 0.0F, 0.0F};
};

#endif // CARPLANT_H
