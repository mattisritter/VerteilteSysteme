#ifndef POSITIONCONTROLLER_H
#define POSITIONCONTROLLER_H

#include <ros/ros.h>
#include "CarParameters.h"

#pragma once
// include STL array type
#include <array>

class PositionController{
public:
  using SigGenInType = std::array<float, 4>; /**< type definition for model input(array of floats with 2 elements) */
  using SigGenOutType = std::array<float, 2>; /**< type definition for model output */
  using FeedForwardInType = std::array<float, 1>; /**< type definition for model input(array of floats with 2 elements) */
  using FeedForwardOutType = std::array<float, 1>; /**< type definition for model output */
  using PosInType = std::array<float, 2>; /**< type definition for model input(array of floats with 2 elements) */
  using PosOutType = std::array<float, 1>; /**< type definition for model output */
  /**
  * @brief Only constructor
  */
  PositionController() // copy parameters to member variables
  {
    //e.fill(0.0F); // initialize input vector with zeros
    //uVpk = 0.0F ;// initialize output with zero
    uVpkDelayed = 0.0F ;// initialize output Delay with zero
    uVpk1Delayed = 0.0F;// initialize input Delay with zero
    actualPosition = 0.0F;
  }


  void init()
  {
  }

  void referenceSignal(const SigGenInType& sigGenIn, const float t, SigGenOutType& sigGenOut )
  {
   //std::array <float, 2> sigGenOut = { 0.0F, 0.0F }; // Array that stores output { wp, uVp1 }
   const float te = 15.0F * sigGenIn.at(2) /  8.0F / sigGenIn.at(1); //Endtime of parking maneuver

   if (t > te)    /*freezes calculation if t > te*/
   {
     sigGenOut.at(0) = sigGenIn.at(2);
     sigGenOut.at(1) = sigGenIn.at(2);
   }
   else
   {
     // Array that stores coefficients for wp
     std::array <float, 6> c = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
     c.at(5) =  65536.0F * sigGenIn.at(1) * sigGenIn.at(1) * sigGenIn.at(1) * sigGenIn.at(1) * sigGenIn.at(1)
                      / sigGenIn.at(2) / sigGenIn.at(2) / sigGenIn.at(2) / sigGenIn.at(2) / 253125.0F;
     c.at(4) = -4096.0F * sigGenIn.at(1) * sigGenIn.at(1) * sigGenIn.at(1) * sigGenIn.at(1)
                      / sigGenIn.at(2) / sigGenIn.at(2) / sigGenIn.at(2) / 3375.0F;
     c.at(3) = 1024.0F * sigGenIn.at(1) * sigGenIn.at(1) * sigGenIn.at(1)
                     / sigGenIn.at(2) / sigGenIn.at(2) / 675.0F;
     c.at(2) = 0.0F;
     c.at(1) = 0.0F;
     c.at(0) = sigGenIn.at(3); //Todo position von einschalten longitudinalregler von y.at() bogenlänge

     sigGenOut.at(0) = c.at(5) * t * t * t * t * t
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

     sigGenOut.at(1) = cff.at(5) * t * t * t * t * t
         + cff.at(4) * t * t * t * t
         + cff.at(3) * t * t * t
         + cff.at(2) * t * t
         + cff.at(1) * t
         + cff.at(0);
   }
   //return sigGenOut;
  }


  void feedForwardControll(const FeedForwardInType& uVpk1, FeedForwardOutType& uVpk)
  {
    //outputsignal
    uVpk.at(0) = uVpkDelayed + (p->Tak + 2 * p->Ti) / 2 * uVpk1.at(0) + (p->Tak + 2 * p->Ti) / 2 * uVpk1Delayed;
    uVpk1Delayed = uVpk1.at(0);
    //return uVpk;
  }

  void positionCtrl(const PosInType& posIn, PosOutType& posOut)
  {
    const float ep = posIn.at(0) - posIn.at(1); //Calculate ep
    posOut.at(0) = p->kp * ep; //Calculate uRp
  }

  float postionCalculation(const float speed, const float time/*, const float x0*/)
  {
    float positionStep = p->Tak * (speed * time + actualPosition) / 2.0F; //Calcluate position with speed and time
    actualPosition += positionStep;
    return actualPosition;
  }

  void startParking()
  {
    actualPosition = 0;
  }

private:
  //Parameters
  float uVpkDelayed;/**< Delay output for one step */
  float uVpk1Delayed;/**< Delay intput for one step */
  float actualPosition;
  float position;
  SigGenInType sigGenIn;
  FeedForwardInType uVpk1;
  PosInType posIn;
  CarParameters* p=CarParameters::p();//Access to Car Parameters
};

#endif // POSITIONCONTROLLER_H
