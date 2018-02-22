#ifndef EV3_ACTION_HPP
#define EV3_ACTION_HPP
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

typedef actionlib::SimpleActionServer<ev3_msgs::DriveServoAction> DriveServoServer;
typedef actionlib::SimpleActionServer<ev3_msgs::DriveServoTimedAction> DriveServoTimedServer;
typedef actionlib::SimpleActionServer<ev3_msgs::DriveSimpleAction> DriveSimpleServer;
typedef actionlib::SimpleActionServer<ev3_msgs::DriveSimpleReverseAction> DriveSimpleReverseServer;
typedef actionlib::SimpleActionServer<ev3_msgs::GripSimpleAction> GripSimpleServer;
typedef actionlib::SimpleActionServer<ev3_msgs::DriveUntilDistanceAction> DriveUntilDistanceServer;

using namespace ev3dev;


class GripSimpleActionServer {
private:
  ros::NodeHandle nh;
  GripSimpleServer gripaction_server;
  medium_motor motor_grip;
  ev3_msgs::GripSimpleFeedback feedback;  
public:
  GripSimpleActionServer(ros::NodeHandle& nh,
    medium_motor& motor_grip
    ): nh(nh), gripaction_server(
        nh, "grip_simple",
        boost::bind(&GripSimpleActionServer::executeGripSimpleCommand, this, _1), false),
      motor_grip(motor_grip){

    }

    void executeGripSimpleCommand(const ev3_msgs::GripSimpleGoalConstPtr &goal);

    void init();

};

class DriveSimpleReverseActionServer {
private:
  ros::NodeHandle nh;
  DriveSimpleReverseServer drivesimplereverse_server;
  large_motor motor_left;
  large_motor motor_right;
  ev3_msgs::DriveSimpleReverseFeedback feedback;
public:
  DriveSimpleReverseActionServer(ros::NodeHandle& nh, 
  large_motor& motor_left, 
  large_motor& motor_right): nh(nh), 
      motor_right(motor_right), motor_left(motor_left), drivesimplereverse_server(
      nh, "drive_simple_reverse",
      boost::bind(&DriveSimpleReverseActionServer::executeDriveSimpleReverseCommand, this, _1), false)
    {
      

  }
  
  void executeDriveSimpleReverseCommand(const ev3_msgs::DriveSimpleReverseGoalConstPtr &goal);

  void init();
  
};

class DriveSimpleActionServer {
private:
  ros::NodeHandle nh;
  DriveSimpleServer drivesimple_server;
  large_motor motor_left;
  large_motor motor_right;
  ev3_msgs::DriveSimpleFeedback feedback;
public:
  DriveSimpleActionServer(ros::NodeHandle& nh, 
  large_motor& motor_left, 
  large_motor& motor_right): nh(nh), 
      motor_right(motor_right), motor_left(motor_left), drivesimple_server(
      nh, "drive_simple",
      boost::bind(&DriveSimpleActionServer::executeDriveSimpleCommand, this, _1), false)
    {
      

  }
  
  void executeDriveSimpleCommand(const ev3_msgs::DriveSimpleGoalConstPtr &goal);

  void init();
  
};

class DriveServoTimedActionServer {
private:
  ros::NodeHandle nh;
  DriveServoTimedServer driveservo_timed_server;
  // Motor
  large_motor     motor_left;
  large_motor     motor_right;
  // Action interface
  ev3_msgs::DriveServoFeedback feedback_;
public:
  DriveServoTimedActionServer(ros::NodeHandle& nh, 
    large_motor& motor_left, large_motor& motor_right): nh(nh), 
    motor_right(motor_right), motor_left(motor_left),
    driveservo_timed_server(
      nh, "drive_servo_timed", 
      boost::bind(&DriveServoTimedActionServer::executeDriveServoTimedCommand, this, _1), false)
      {
    
  }
  void executeDriveServoTimedCommand(const ev3_msgs::DriveServoTimedGoalConstPtr &goal);

  void init();
};

class DriveUntilDistanceActionServer {
private:
  ros::NodeHandle nh;
  DriveUntilDistanceServer driveuntildistance_server;
  large_motor motor_left;
  large_motor motor_right;
  int distance;
  ros::Subscriber sub;
  ev3_msgs::DriveUntilDistanceFeedback feedback;
public:
  DriveUntilDistanceActionServer(ros::NodeHandle& nh,
    large_motor& motor_left, 
    large_motor& motor_right
    ): nh(nh), motor_right(motor_right), motor_left(motor_left),
    driveuntildistance_server(
      nh, "drive_until_distance",
      boost::bind(&DriveUntilDistanceActionServer::execute, this, _1), false){
    
  }

  void execute(const ev3_msgs::DriveUntilDistanceGoalConstPtr &goal);

  void cb_ir(const ev3_msgs::SensorIRConstPtr ir);

  void init();
};

#endif