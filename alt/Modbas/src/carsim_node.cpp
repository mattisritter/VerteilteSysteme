#include"carplant.h"

// include the interface of the general ROS C++ library
#include <ros/ros.h>

#include "madmsgs/CarInputs.h"
#include "madmsgs/CarOutputs.h"
#include "madmsgs/CarOutputsExt.h"

/**
 * @brief The CarSimNode class
 */
class CarSimNode{

public:
  // the following members are visible to outside world
  using ModelType = CarPlant;
  /**
* @brief The only constructor which is called on class instantiation
* @param[in] samplingTimeArg The sample time [s]
*/
  CarSimNode(const float samplingTimeArg) :
    samplingTime(samplingTimeArg) // initializes the member samplingTime
  {
    // create subscriber for u
    inputsSub = node.subscribe("/mad/carinputs", 1, &CarSimNode::inputsCallback, this);
    // create publisher for y
    outputsExtPub = node.advertise<madmsgs::CarOutputsExt>("/mad/car0/sim/caroutputsext", 1);
    outputsPub = node.advertise<madmsgs::CarOutputs>("/mad/caroutputs", 1);
  }
  /**
* @brief step function to execute one sampling step
*/
  void step()
  {
    ros::Time time { ros::Time::now() };
    // create ROS message of type Float32
    madmsgs::CarOutputsExt carOutputsExtMsg;
    madmsgs::CarOutputs carOutputsMsg;
    // compute one sampling point of state space model
    ModelType::OutputsType y;
    model.step(u, y, samplingTime);

    carOutputsExtMsg.header.stamp = time;
    carOutputsMsg.header.stamp = time;

    // assign the output signal to the message
    carOutputsExtMsg.s = { y.at(0), y.at(1) };
    carOutputsMsg.s = { y.at(0), y.at(1) };
    carOutputsExtMsg.psi = y.at(2);
    carOutputsMsg.psi = y.at(2);
    carOutputsExtMsg.beta = y.at(3);
    carOutputsExtMsg.v = y.at(4);
    carOutputsExtMsg.x = y.at(5);

    // output the message on the topic /CarOutsputsExt and /CarOutsputs
    outputsExtPub.publish(carOutputsExtMsg);

    if (counterCarOutputs >= 10) /*message is published every 10ms*/
    {
      outputsPub.publish(carOutputsMsg);
      counterCarOutputs = 0;
    }
    counterCarOutputs++;

  }
private:
  // the following members are not visible to the outside of this class
  ros::NodeHandle node { "~" }; /**< The ROS node handle. */
  // Note: with optional constructor argument "~"
  // all topic names are relative to the node name
  ros::Subscriber inputsSub; /**< The /mad/carinputs topic subscriber */
  ros::Publisher outputsExtPub; /**< The /mad/car0/sim/caroutputsext topic publisher */
  ros::Publisher outputsPub; /**< The /mad/caroutputs topic publisher */
  const float samplingTime = 0.0F; /**< The sample time [s] */
  ModelType::InputsType u { { 0.0F, 0.0F } }; /**< The input signal of the model */
  ModelType model {};

  int8_t counterCarOutputs = 0; // counter for sending Message every 10ms
  /**
* @brief callback for u topic, u is the input to the model
* @param[in] msg The ROS message
*/
  void inputsCallback(const madmsgs::CarInputs& msg)
  {
    // copy the input signal to the member variable u
    // limit pedal input depending on the command message
    switch (msg.cmd)
    {
    case madmsgs::CarInputs::CMD_HALT:
      // set pedal input to 0
      u.at(0) = 0.0F;
      break;

    case madmsgs::CarInputs::CMD_FORWARD:
      // limit pedal input to [0, 1]
      if (msg.pedals > 1.0F)
      {
        u.at(0) = 1.0F;
      }
      else if (msg.pedals < 0.0F)
      {
        u.at(0) = 0.0F;
      }
      else
      {
        u.at(0) = msg.pedals;
      }
      break;

    case madmsgs::CarInputs::CMD_REVERSE:
      // limit pedal input to [-1, 0]
      if (msg.pedals < -1.0F)
      {
        u.at(0) = -1.0F;
      }
      else if (msg.pedals > 0.0F)
      {
        u.at(0) = 0.0F;
      }
      else
      {
        u.at(0) = msg.pedals;
      }
      break;

    case madmsgs::CarInputs::CMD_SLOW:
      // pass pedal input
      u.at(0) = msg.pedals;
      break;

    default:
      // Backup: if no siganl received, car supposed to stop
      u.at(0) = 0.0F;
      break;
    }
    // limit steering input to [-1, 1]
    if (msg.steering > 1.0F)
    {
      u.at(1) = 1.0F;
    }
    else if (msg.steering < -1.0F)
    {
      u.at(1) = -1.0F;
    }
    else
    {
      u.at(1) = msg.steering;
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
  // instantiate class CarSimNode and
  // call CarSimNode(const float samplingTimeArg) constructor
  CarSimNode node(samplingTime);
  // define sampling rate as the inverse of the sample time
  ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));

  // loop while ROS is running
  while (ros::ok()) {
    // call the method step() of the CarsimNode instance node
    node.step();
    // pass control to ROS for background tasks
    ros::spinOnce();
    // wait for next sampling point
    // neighbor sampling points have a time distance of 2ms
    loopRate.sleep();
  }
  // return success
  return EXIT_SUCCESS;
}

