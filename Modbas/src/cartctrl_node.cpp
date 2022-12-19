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
    std_msgs::Float32 testMsg;
    //output variables
    // Speed Controller
    float uv; // control signal for pedals
    // Position Controller
    float up; // control signal
    float uVp; // output of feed forward control
    float uRp; // output of controller
    // Path Controller
    float delta; // control signal for steering

    // Path Controller (enabled as soon as there is a drive maneuver message)
    if (type != madmsgs::DriveManeuver::TYPE_HALT)
    {
      pathController.referenceSignal(y, vs, spline); // generate reference signals
      pathController.pathCtrl(y, psi, vs); // feedback state-space controller
      pathController.feedforwardCtrl(); // feedforward controller
      pathController.step(delta); // calculate and limit delta

      carInputsMsg.steering = delta;
      wv = vs; // set reference signal for speed controller to max speed of maneuver

      //Position Controller
      if (type == madmsgs::DriveManeuver::TYPE_PARK)
      {
        positionController.referenceSignal(xs, vs, x0, t, wp, uVp1k); //Calulate Reference Signals
        positionController.feedforwardCtrl(uVp1k, uVp1k_1, uVp); //Online feed forward control
        ep = wp - x;
        positionController.positionCtrl(ep, uRp);
        up = uVp + uRp; //Calculate control signal for position control
        wv = up; //set reference signal for speed controller to control signal of position controller
      }
      else
      {
        wp = x;
        uVp1k_1 = uVp1k;
        uVp1k = x;
      }
    }

    //Speed Controller (always active)
    ev = wv - v; //calculation of control deviation for speed
    speedController.step(ev, uv);
    carInputsMsg.pedals = uv; // copy the output signal to the message


    //creating cmd message
    //Drive mode parking
    if (type == madmsgs::DriveManeuver::TYPE_PARK)
     {
        carInputsMsg.cmd = madmsgs::CarInputs::CMD_SLOW;
     }
    //Drive mode forward
     else if (wv >= 0.0F)
    {
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_FORWARD;
    }
    //Drive mode reverse
    else
    {
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_REVERSE;
    }
    testMsg.data = x; //x
    debugMsg.data = wp; //wp

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
  //car outputs
  float v = 0.0F; //actual speed of the rear axle [m/s]
  float x = 0.0F; //actual arcposition [m] (calculated as integral of v)
  float v_1 = v; //old sped for integration
  boost::array<float, 2> y {{0.0F, 0.0F}}; //cartesian center position [ m ]
  float psi = 0.0F; //yaw angle [rad]
  //drive maneuver
  uint type = 0; //type of drive maneuver
  float vs = 0.0F; //max speed [m/s]
  float xs = 0.0F; //end of park maneuver [m]
  std::vector<float> breaks { 0.0F };
  std::array<std::vector<float>, 2> s { {{0.0F}, {0.0F}} };
  std::vector<uint32_t> segmentIDs { 0 };
  // Speed Controller
  float wv = 0.0F; // reference speed [m/s]
  float ev = 0.0F; // control deviation [m/s]
  // Position Controller
  float wp = 0.0F; // reference signal [m]
  float uVp1k = 0.0F; // time discrete reference signal for feed forward control
  float uVp1k_1 = 0.0F; // last value of uVp1k
  float ep = 0.0F; // control deviation [m]
  float x0 = 0.0F; // start arcposition [m]
  float t = 0.0F;   // trace elapsed time [s]
  // Path Controller
  modbas::Spline spline; // default constructor of a spline

  // construct controllers
  SpeedControllerType speedController {};
  PositionControllerType positionController {};
  PathControllerType pathController {};

  /**
 * @brief integration of speed v [m/s] to calculate arcposition x [m]
 * @param[in] v speed [m/s]
 */
  void calculateX(const float v)
  {
    x += CarParameters::p()->Tak * (v + v_1) / 2.0F; // trapezodial rule
    v_1 = v;
  }
  /**
 * @brief callback for CarOutputsExt
 * @param[in] msgOutputsExt the ROS message
 */
  void inputsCallbackOutputsExt(const madmsgs::CarOutputsExt& msgOutputsExt)
  {
    v = msgOutputsExt.v; // actual speed v [m/s]
    calculateX(v); // calculate arcposition x [m] as integral of v
    psi = msgOutputsExt.psi; // actual yaw angle [rad]
    y = msgOutputsExt.s; // actual cartesian center position [m] (s1,s2)T

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
    // Position Controller
    t = 0.0F; // reset time [s]
    x0 = x; // set x0 [m] to current arcposition
    // Path Controller
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

