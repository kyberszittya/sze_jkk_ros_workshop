#include <thread>
#include <chrono>
#include <cmath>
// EV3DEV
#include "ev3dev.h"
// ROS
#include <ros/ros.h>
// Actions
#include <ev3_msgs/DriveServoAction.h>
#include <ev3_msgs/DriveServoTimedAction.h>
#include <ev3_msgs/DriveSimpleAction.h>
#include <ev3_msgs/DriveSimpleReverseAction.h>
#include <ev3_msgs/GripSimpleAction.h>
#include <ev3_msgs/GripSimpleFeedback.h>
#include <ev3_msgs/DriveSimpleReverseFeedback.h>
#include <ev3_msgs/DriveSimpleFeedback.h>
#include <ev3_msgs/DriveUntilDistanceAction.h>
#include <ev3_msgs/DriveUntilDistanceFeedback.h>
// Actionlib
#include <actionlib/server/simple_action_server.h>
// Sensor Msgs
#include <ev3_msgs/SensorIR.h>
// Geom msgs
#include <geometry_msgs/Pose2D.h>
// Std msgs
#include <chrono>

#include "ev3_controller.hpp"

// Heartbeat
//#include <bondcpp/bond.h>



int main(int argc, char** argv)
{  
  ros::init(argc, argv, "ev3_controller");
  ros::NodeHandle nh;
  /*
  std::string id = "ev3c";
  bond::Bond bond("ev3_component_heartbeat", id);
  ROS_INFO("Starting heartbeat");
  bond.start();
  if (!bond.waitUntilFormed(ros::Duration(60.0)))
  {
    ROS_ERROR("Nothing received back from sensor module");
    return 0;
  }
  ROS_INFO("Heartbeat received");
  */
  EV3Controller ev3control(nh);
  ev3control.init();
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
