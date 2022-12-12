#include "speedcontroller.h"
#include "positioncontroller.h"
#include "pathcontroller.h"

// include the interface of the general ROS C++ library
#include <ros/ros.h>
// include the data type of the standard ROS message Float32
#include <std_msgs/Float32.h>

#include "madmsgs/CarInputs.h"
#include "madmsgs/CarOutputsExt.h"
#include "madmsgs/DriveManeuver.h"


class CarCtrlNode{

public:
  // The following members are visible to outside world
  using SpeedControllerType = SpeedController;
  using PositionControllerType = PositionController;
  using PathControllerType = PathController;
  /**
 * @brief The only constructor which is called on class instantiation
 * @param[in] samplingTimeArg The sample time [ s ]
 */
  CarCtrlNode(const float samplingTimeArg) :
    samplingTime(samplingTimeArg) // initializes the member samplingTime
  {
    // create subscriber for outputsExtSub and driveManeuverSub
    outputsExtSub = node.subscribe("/mad/caroutputsext", 1, &CarCtrlNode::inputsCallbackOutputsExt, this);
    driveManeuverSub = node.subscribe("/mad/car0/navi/maneuver", 1, &CarCtrlNode::inputsCallbackDriveManeuver, this);
    // create publisher for inputsPub
    inputsPub = node.advertise<madmsgs::CarInputs>("/mad/carinputs", 1);
    debugPub = node.advertise<std_msgs::Float32>("/mad/car0/ctrl/debug/wp", 1);
    testPub = node.advertise<std_msgs::Float32>("/mad/car0/ctrl/test/x", 1);
    //Initiate position tracking
    positionController.startParking(e.at(0));
  }
  /**
 * @brief step function to execute one sampling step
 */
  void step()
  {
    // create ROS message of type Float32
    madmsgs::CarInputs carInputsMsg;
    // create ROS messages of type Float32
    std_msgs::Float32 debugMsg;
    std_msgs::Float32 testMsg;
    // compute one sampling point of state space model
    SpeedControllerType::OutputsType uk;
    PositionControllerType::SigGenOutType sigGenOut;
    PositionControllerType::FeedForwardOutType uVpk;
    PositionControllerType::PosOutType uRp;
    //Todo pathcontroller output

    //Calculate current arc Position
    arcPosition = positionController.postionCalculation(e.at(0));

    //Parkposition
    if (sigGenIn.at(0) == madmsgs::DriveManeuver::TYPE_PARK) /*Uses Feed forward controll signal*/
    {
      positionController.referenceSignal(sigGenIn, t, sigGenOut); //Calulate Reference Signals
      uVpk1.at(0) = sigGenOut.at(1);
      positionController.feedForwardControll(uVpk1, uVpk); //Online feed forward control
      ep.at(0) = sigGenOut.at(0);
      ep.at(1) = arcPosition;

      testMsg.data = ep.at(1);
      debugMsg.data = sigGenOut.at(0);

      positionController.positionCtrl(ep, uRp);
      e.at(1) = uVpk.at(0) + uRp.at(0);//Calculate up = uVpk + uRp;
    }

    speedController.step(e, uk);
    // copy the output signal to the message
    if (uk.at(0) >= 1.0F)       /*limit control signal to 1*/
    {
      carInputsMsg.pedals = 1.0F;
    }
    else if (uk.at(0) <= -1.0F) /*limit controll signal to -1*/
    {
      carInputsMsg.pedals = -1.0F;
    }
    else                        /*Pass controll signal*/
    {
      carInputsMsg.pedals = uk.at(0);
    }

    //creating cmd message
    //Drive mode parking
    if (sigGenIn.at(0) == madmsgs::DriveManeuver::TYPE_PARK)
     {
        carInputsMsg.cmd = madmsgs::CarInputs::CMD_SLOW;
     }
    //Drive mode forward
     else if (e.at(1) >= 0.0F)
    {
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_FORWARD;
    }
    //Drive mode reverse
    else
    {
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_REVERSE;
    }

    //Constant steering angle
    carInputsMsg.steering = 0.7F; //Value according to book

    // output the message on the topic /carinputs
    inputsPub.publish(carInputsMsg);
    debugPub.publish(debugMsg);
    testPub.publish(testMsg);

    t += CarParameters::p()->Tak;    //Increase elapsed time
  }

private:
  // The following members are not visible to the outside of this class
  ros::NodeHandle node { "~" }; /**< The ROS node handle. */
  // Note: with optional constructor argument "~"
  // all topic names are relative to the node name
  ros::Subscriber outputsExtSub; /**< The /mad/caroutputsext topic subscriber */
  ros::Subscriber driveManeuverSub; /**< The /mad/car0/navi/maneuver topic subscriber */
  ros::Publisher inputsPub; /**< The /mad/carinputs topic publisher */
  ros::Publisher debugPub; /**< The /mad/car0/ctrl/debug/wp topic publisher */
  ros::Publisher testPub;
  const float samplingTime = 0.0F; /**< The sample time [s] */
  SpeedControllerType::InputsType e { { 0.0F, 0.0F } }; /**< The input signal of the model */
  PositionControllerType::SigGenInType sigGenIn { { 0.0F, 0.0F, 0.0F, 0.0F } };
  PositionControllerType::FeedForwardInType uVpk1 { 0.0F };
  PositionControllerType::PosInType ep { {0.0F, 0.0F}};
  PathControllerType::PathInSType pathInS { { { 0.0F } , { 0.0F }, { 0.0F } } };
  float vmax = 0.0F;
  boost::array<float, 2> s { { 0.0F, 0.0F } };
  std::vector<uint32_t> segmentIds { { 0 } };
  PathControllerType::PathInAType pathInA { {0.0F, 0.0F, 0.0F, 0.0F } };
  float t = 0.0F;   //Trace elapsed time
  float arcPosition = 0.0F; //Actual position
  SpeedControllerType speedController {};
  PositionControllerType positionController {};

  /**
 * @brief callback for u topic, u is the input to the model
 * @param[in] msg The ROS message
 */
  void inputsCallbackOutputsExt(const madmsgs::CarOutputsExt& msgOutputsExt)
  {
    // copy the input signal to the member variable e
    e.at(0) = msgOutputsExt.v;        //Actual speed y
    // copy the input signal to the member variable pathInA
    pathInA.at(0) = msgOutputsExt.v;
    //boost::array<float, 2> test = msgOutputsExt.s;
    s = msgOutputsExt.s;
    //pathInA.at(2) = test.at(1);
    pathInA.at(3) = msgOutputsExt.psi;
  }
  void inputsCallbackDriveManeuver(const madmsgs::DriveManeuver& msgDriveManeuver)
  {
    // copy the input signal to the member variable e
    e.at(1) = msgDriveManeuver.vmax;  //Wished speed w
    // copy the input signal to the member variable sigGenIn
    sigGenIn.at(0) = msgDriveManeuver.type;  //Type of maneuver
    sigGenIn.at(1) = msgDriveManeuver.vmax;  //Wished speed w
    sigGenIn.at(2) = msgDriveManeuver.xManeuverEnd; //target arc length in case of PARK
    // copy the input signal to the member variable pathInS
    vmax = msgDriveManeuver.vmax;
    pathInS.at(0) = msgDriveManeuver.breaks;
    pathInS.at(1) = msgDriveManeuver.s1;
    pathInS.at(2) = msgDriveManeuver.s2;
    segmentIds = msgDriveManeuver.segments;

    //Start Parameters of Parking maneuver
    t = 0.0F;
    sigGenIn.at(3) = arcPosition; //Set x0 to current arc Position
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

