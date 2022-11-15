#include"carplant.h"

// include the interface of the general ROS C++ library
#include <ros/ros.h>
// include the data type of the standard ROS message Float32
//#include <std_msgs/Float32.h>

#include "madmsgs/CarInputs.h"
#include "madmsgs/CarOutputs.h"
#include "madmsgs/CarOutputsExt.h"

class CarSimNode{
 // CarSimNode(){};
public:
  // The following members are visible to outside world
  //using ModelType = Pt1model;
  using ModelType = CarPlant;
  /**
* @brief The only constructor which is called on class instantiation
* @param[in] samplingTimeArg The sample time [ s ]
*/
  CarSimNode(const float samplingTimeArg) :
    samplingTime(samplingTimeArg) // initializes the member samplingTime
  {
    // create subscriber for u
    inputsSub = node.subscribe("/mad/carinputs", 20, &CarSimNode::inputsCallback, this);
    // create publisher for y
    outputsExtPub = node.advertise<madmsgs::CarOutputsExt>("/mad/car0/sim/caroutputsext", 2);
    outputsPub = node.advertise<madmsgs::CarOutputs>("/mad/caroutputs", 20);

    // test the C++ functor object (can be commented out)
    // a functor object behaves like a C++ function
    // in this case: the right-hand side of the differential equation
    /*Todo->    ModelType::StatesType x { 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F };
    ModelType::StatesType xd;
    model(x, xd, 0.0F);
    ROS_INFO("The functor object returned xd[0]=%12.8g, xd[1]=%12.8g, xd[2]=%12.8g, xd[3]=%12.8g, xd[4]=%12.8g, xd[5]=%12.8g, xd[6]=%12.8g", xd.at(0), xd.at(1), xd.at(2), xd.at(3), xd.at(4), xd.at(5), xd.at(6));
    //<-
    */
  }
  /**
* @brief step function to execute one sampling step
*/
  void step()
  {
    // create ROS message of type Float32
    madmsgs::CarOutputsExt carOutputsExtMsg;
    madmsgs::CarOutputs carOutputsMsg;
    // compute one sampling point of state space model
    ModelType::OutputsType y;
    model.step(u, y, samplingTime);
    // copy the output signal to the message
    carOutputsExtMsg.s = {y.at(0), y.at(1)};
    carOutputsMsg.s = {y.at(0), y.at(1)};
    carOutputsExtMsg.psi = y.at(2);
    carOutputsMsg.psi = y.at(2);
    carOutputsExtMsg.beta = y.at(3);
    carOutputsExtMsg.v = y.at(4);
    carOutputsExtMsg.x = y.at(5);
    // output the message on the topic /CarOutsputsExt and /CarOutsputs
    outputsExtPub.publish(carOutputsExtMsg);
    outputsPub.publish(carOutputsMsg);
  }
private:
  // The following members are not visible to the outside of this class
  ros::NodeHandle node { "~" }; /**< The ROS node handle. */
  // Note: with optional constructor argument "~"
  // all topic names are relative to the node name
  ros::Subscriber inputsSub; /**< The /mad/carinputs topic subscriber */
  ros::Publisher outputsExtPub; /**< The /mad/car0/sim/caroutputsext topic publisher */
  ros::Publisher outputsPub; /**< The /mad/caroutputs topic publisher */
  const float samplingTime = 0.0F; /**< The sample time [s] */
  ModelType::InputsType u { { 0.0F, 0.0F, 0.0F} }; /**< The input signal of the model */
  ModelType model {};
  /**
* @brief callback for u topic, u is the input to the model
* @param[in] msg The ROS message
*/
  void inputsCallback(const madmsgs::CarInputs& msg)
  {
    // copy the input signal to the member variable u
    u.at(0) = msg.cmd;
    switch(msg.cmd)
    {
    case madmsgs::CarInputs::CMD_HALT:
      u.at(1) = 0;
      break;

    case madmsgs::CarInputs::CMD_FORWARD:
      if(msg.pedals > 1)
      {
        u.at(1) = 1;
      }
      else if(msg.pedals < 0)
      {
        u.at(1) = 0;
      }
      else
      {
        u.at(1) = msg.pedals;
      }
      break;

    case madmsgs::CarInputs::CMD_REVERSE:
      if(msg.pedals < -1)
      {
        u.at(1) = -1;
      }
      else if(msg.pedals > 0)
      {
        u.at(1) = 0;
      }
      else
      {
        u.at(1) = msg.pedals;
      }
      break;

    case madmsgs::CarInputs::CMD_SLOW:
        /*Todo: Frage ob bei langsamfahrt pedal signal weiter gegeben werden kann*/
      u.at(1) = msg.pedals;
      break;

    default: /*Backup if no siganl received, car supposed to stop*/
      u.at(1) = 0;
      break;
    }

    if(msg.steering > 1)
    {
      u.at(2) = 1;
    }
    else if(msg.steering < -1)
    {
      u.at(2) = -1;
    }
    else
    {
      u.at(2) = msg.steering;
    }
  }
};


/**
* @brief The main function.
* @param[in] argc number of arguments
* @param[in] argv C array of string pointers to arguments
* @return 0 on success
*/
int main(int argc, char **argv)
{
  const float samplingTime = CarParameters::p()->Ta; // the constant sample time [ s ]
  ros::init(argc, argv, "carsim_node"); // initialize ROS
  // instantiate class SineNode and
  // call SineNode(const float samplingTimeArg) constructor
  CarSimNode node(samplingTime);
  // define sampling rate as the inverse of the sample time
  ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));
  // loop while ROS is running
  while (ros::ok()) {
    // call the method step() of the SineNode instance node
    node.step();
    // pass control to ROS for background tasks
    ros::spinOnce();
    // wait for next sampling point
    // neighbor sampling points have a time distance of 100ms
    loopRate.sleep();
  }
  // return success
  return EXIT_SUCCESS;
}

