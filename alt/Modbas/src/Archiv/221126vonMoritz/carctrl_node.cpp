#include"speedcontroller.h"

// include the interface of the general ROS C++ library
#include <ros/ros.h>
// include the data type of the standard ROS message Float32
//#include <std_msgs/Float32.h>

#include "madmsgs/CarInputs.h"
//#include "madmsgs/CarOutputs.h"
#include "madmsgs/CarOutputsExt.h"
#include "madmsgs/DriveManeuver.h"

class CarCtrlNode{

public:
  // The following members are visible to outside world
  using ModelType = SpeedController;
  /**
 * @brief The only constructor which is called on class instantiation
 * @param[in] samplingTimeArg The sample time [ s ]
 */
  CarCtrlNode(const float samplingTimeArg) :
    samplingTime(samplingTimeArg) // initializes the member samplingTime
  {
    // create subscriber for u
    outputsExtSub = node.subscribe("/mad/caroutputsext", 1, &CarCtrlNode::inputsCallbackOutputsExt, this);
    driveManeuverSub = node.subscribe("/mad/car0/navi/maneuver", 1, &CarCtrlNode::inputsCallbackDriveManeuver, this);
    // create publisher for y
    inputsPub = node.advertise<madmsgs::CarOutputsExt>("/mad/carinputs", 1);

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
    madmsgs::CarInputs carInputsMsg;
    // compute one sampling point of state space model
    ModelType::OutputsType uk;
    model.step(e, uk/*, samplingTime*/);
    // copy the output signal to the message
    if (uk.at(0) >= 1.0F)       /*limit control signal to 1*/
    {
      carInputsMsg.pedals = 1.0F;
    }
    else if (uk.at(0) <= -1.0F) /*limit controll signal to -1*/
    {
      carInputsMsg.pedals = -1.0F;
    }
    else                     /*Pass controll signal*/
    {
      carInputsMsg.pedals = uk.at(0);
    }

    //creating cmd message
    //Todo: einbinden des fahrmodus slow
    /*if (Longitudinalpos regler an)
     {
        carInputsMsg.cmd = madmsgs::CarInputs::CMD_SLOW;
     }
     else */if (e.at(1) >= 0.0F)
    {
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_FORWARD;
    }
    else
    {
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_REVERSE;
    }

    //Constant steering angle
    carInputsMsg.steering = 0.7F; //Value according to book

    // output the message on the topic /carinputs
    if (counterCarOutputs >= 19)/*Message is published every 20ms*/
    {
      inputsPub.publish(carInputsMsg);
      counterCarOutputs = 0;
    }
    counterCarOutputs++;
  }

private:
  // The following members are not visible to the outside of this class
  ros::NodeHandle node { "~" }; /**< The ROS node handle. */
  // Note: with optional constructor argument "~"
  // all topic names are relative to the node name
  ros::Subscriber outputsExtSub; /**< The /mad/caroutputsext topic subscriber */
  ros::Subscriber driveManeuverSub; /**< The /mad/car0/navi/maneuver topic subscriber */
  ros::Publisher inputsPub; /**< The /mad/carinputs topic publisher */
  const float samplingTime = 0.0F; /**< The sample time [s] */
  ModelType::InputsType e { { 0.0F, 0.0F } }; /**< The input signal of the model */
  ModelType model {};

  int counterCarOutputs = 0;//Counter for releasing Message every 20ms
  /**
 * @brief callback for u topic, u is the input to the model
 * @param[in] msg The ROS message
 */
  void inputsCallbackOutputsExt(const madmsgs::CarOutputsExt& msgOutputsExt/*, const madmsgs::DriveManeuver& msgDriveManeuver*/)
  {
    // copy the input signal to the member variable u
    e.at(0) = msgOutputsExt.v;        //Actual speed y
    //e.at(1) = msgDriveManeuver.vmax;  //Wished speed w
  }
  void inputsCallbackDriveManeuver(/*const madmsgs::CarInputs& msgOutputsExt,*/ const madmsgs::DriveManeuver& msgDriveManeuver)
  {
    // copy the input signal to the member variable u
    //e.at(0) = msgOutputsExt.v;        //Actual speed y
    e.at(1) = msgDriveManeuver.vmax;  //Wished speed w
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
  const float samplingTime = CarParameters::p()->Tak; // the constant sample time [ s ]
  ros::init(argc, argv, "carctrl_node"); // initialize ROS
  // instantiate class CarCtrlNode and
  // call CarCtrlNode(const float samplingTimeArg) constructor
  CarCtrlNode node(samplingTime);
  // define sampling rate as the inverse of the sample time
  ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));
  //Initialisation node
  //node.init();

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

