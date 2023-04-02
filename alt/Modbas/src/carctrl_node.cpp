#include "speedcontroller.h"
#include "positioncontroller.h"
#include "pathcontroller.h"

#include "Spline.h"

// include the interface of the general ROS C++ library
#include <ros/ros.h>
// include the data type of the standard ROS message Float32
#include <std_msgs/Float32.h>

#include "madmsgs/CarInputs.h"
#include "madmsgs/CarOutputsExt.h"
#include "madmsgs/DriveManeuver.h"

/**
 * @brief The CarCtrlNode class
 */
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
  CarCtrlNode()
  {
    // create subscriber for outputsExtSub and driveManeuverSub
    outputsExtSub = node.subscribe("/mad/caroutputsext", 1, &CarCtrlNode::inputsCallbackOutputsExt, this);
    driveManeuverSub = node.subscribe("/mad/car0/navi/maneuver", 1, &CarCtrlNode::inputsCallbackDriveManeuver, this);
    // create publisher for inputsPub, debugPub and testPub
    inputsPub = node.advertise<madmsgs::CarInputs>("/mad/carinputs", 1);
    debugPub = node.advertise<std_msgs::Float32>("/mad/car0/ctrl/debug/wp", 1);
    uvpPub = node.advertise<std_msgs::Float32>("/mad/car0/ctrl/debug/uvp", 1);
    wvPub = node.advertise<std_msgs::Float32>("/mad/car0/ctrl/debug/wv", 1);
    testPub = node.advertise<std_msgs::Float32>("/mad/car0/ctrl/test/x", 1);
  }
  /**
 * @brief step function to execute one sampling step
 */
  void step()
  {
    // create ROS message of type CarInputs
    madmsgs::CarInputs carInputsMsg;
    // create ROS messages of type Float32 for debugging
    std_msgs::Float32 debugMsg;
    std_msgs::Float32 uvpMsg;
    std_msgs::Float32 wvMsg;
    std_msgs::Float32 testMsg;
    //output variables
    calculateX(v, x); // calculate arcposition x [m] as integral of v
    // Speed Controller
    float uv = 0.0F; // control signal for pedals
    // Position Controller
    float up = 0.0F; // control signal
    // Path Controller
    float delta = 0.0F; // control signal for steering

    // Path Controller (enabled as soon as there is a drive maneuver message)
    if (type != madmsgs::DriveManeuver::TYPE_HALT)
    {
      pathController.step(spline, y, psi, vs, delta);
      // set reference signal for speed controller to max speed of maneuver
      wv = vs;
      carInputsMsg.steering = delta;
    }

    //Position Controller
    if (type == madmsgs::DriveManeuver::TYPE_PARK)
    {
      positionController.step(xs, vs, x, up);
      wv = up; //set reference signal for speed controller to control signal of position controller
    }

    // Speed Controller (always active)
    speedController.step(wv, v, uv);
    carInputsMsg.pedals = uv; // copy the output signal to the message

    // creating cmd message
    // Drive mode forward
    if (wv >= 0.0F)
    {
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_FORWARD;
    }
    // Drive mode reverse
    else
    {
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_REVERSE;
    }
    // Drive mode parking
    if (type == madmsgs::DriveManeuver::TYPE_PARK)
    {
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_SLOW;
    }


    testMsg.data = x; // x
    debugMsg.data = positionController.getwp(); // wp
    wvMsg.data = wv;
    uvpMsg.data = positionController.getuvp1();

    // output the message on the topic /carinputs
    inputsPub.publish(carInputsMsg);
    debugPub.publish(debugMsg);
    uvpPub.publish(uvpMsg);
    wvPub.publish(wvMsg);
    testPub.publish(testMsg);
  }

private:
  // The following members are not visible to the outside of this class
  ros::NodeHandle node { "~" }; /**< The ROS node handle. */
  // Note: with optional constructor argument "~"
  // all topic names are relative to the node name
  ros::Subscriber outputsExtSub; /**< The /mad/caroutputsext topic subscriber */
  ros::Subscriber driveManeuverSub; /**< The /mad/car0/navi/maneuver topic subscriber */
  ros::Publisher inputsPub; /**< The /mad/carinputs topic publisher */
  ros::Publisher uvpPub;
  ros::Publisher wvPub;
  ros::Publisher debugPub; /**< The /mad/car0/ctrl/debug/wp topic publisher */
  ros::Publisher testPub; /**< The /mad/car0/ctrl/test/x topic publisher */
  //car outputs
  float v = 0.0F; // actual speed of the rear axle [m/s]
  float x = 0.0F; // actual arcposition [m] (calculated as integral of v)
  float vm1 = 0.0F; // old speed for integration
  boost::array<float, 2> y { {0.0F, 0.0F} }; // cartesian center position [m]
  float psi = 0.0F; //yaw angle [rad]
  //drive maneuver
  uint type = 0; // type of drive maneuver
  float vs = 0.0F; // max speed [m/s]
  float xs = 0.0F; // end of park maneuver [m]
  std::vector<float> breaks { 0.0F };
  std::array<std::vector<float>, 2> s { { { 0.0F }, { 0.0F } } }; // actual cartesian center position [m] (s1, s2)^T
  std::vector<uint32_t> segmentIDs { 0 };
  // Speed Controller
  float wv = 0.0F; // reference speed [m/s]
  // Path Controller
  modbas::Spline spline; // default constructor of a spline

  // construct controllers
  SpeedControllerType speedController {};
  PositionControllerType positionController {};
  PathControllerType pathController {};

  /**
 * @brief integration of speed v [m/s] to calculate arcposition x [m]
 * @param[in] v speed [m/s]
 * @param[out] x arcposition [m]
 */
  void calculateX(const float v, float& x)
  {
    x += CarParameters::p()->Tak * (v + vm1) / 2.0F; // trapezodial rule
    vm1 = v;
  }
  /**
 * @brief callback for CarOutputsExt
 * @param[in] msgOutputsExt the ROS message
 */
  void inputsCallbackOutputsExt(const madmsgs::CarOutputsExt& msgOutputsExt)
  {
    v = msgOutputsExt.v; // actual speed v [m/s]
    psi = msgOutputsExt.psi; // actual yaw angle [rad]
    y = msgOutputsExt.s; // actual cartesian center position [m] (s1, s2)^T
  }

  /**
 * @brief callback for DriveManeuver
 * @param[in] msgDriveManeuver the ROS message
 */
  void inputsCallbackDriveManeuver(const madmsgs::DriveManeuver& msgDriveManeuver)
  {
    vs = msgDriveManeuver.vmax;  // max speed [m/s]
    type = msgDriveManeuver.type;  //T ype of maneuver
    xs = msgDriveManeuver.xManeuverEnd; // target arc length [m] in case of PARK
    // waypoints for spline gerneration
    breaks = msgDriveManeuver.breaks;
    s.at(0) = msgDriveManeuver.s1;
    s.at(1) = msgDriveManeuver.s2;
    segmentIDs = msgDriveManeuver.segments;
    // initialize position Controller
    positionController.init(x, xs, vs);
    // initialize path Controller
    pathController.init(breaks, s, segmentIDs, spline);
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
  CarCtrlNode node;
  // define sampling rate as the inverse of the sample time
  ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));
  //Initialisation node
  //node.init();

  // loop while ROS is running
  while (ros::ok()) {
    // call the method step() of the CarctrlNode instance node
    node.step();
    // pass control to ROS for background tasks
    ros::spinOnce();
    // wait for next sampling point
    // neighbor sampling points have a time distance of 20ms
    loopRate.sleep();
  }
  // return success
  return EXIT_SUCCESS;
}

