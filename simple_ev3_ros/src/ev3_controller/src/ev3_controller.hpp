#ifndef EV3_CONTROLLER_HPP
#define EV3_CONTROLLER_HPP
#include "ev3_actions.hpp"

using namespace ev3dev;

class EV3Controller
{
private:
  // Robot parameters
  float r_wheels;
  // ROS
  ros::NodeHandle nh;
  // EV3 interface
  large_motor     motor_left;
  large_motor     motor_right;
  medium_motor    motor_grip;
  
  // Action servers
  DriveServoTimedActionServer drive_servo_ac;
  DriveSimpleActionServer simple_drive_ac;
  DriveSimpleReverseActionServer simple_reverse_ac;
  GripSimpleActionServer grip_simple_ac;
  DriveUntilDistanceActionServer driveuntildist_ac;
public:

  EV3Controller(ros::NodeHandle& nh_): nh(nh_),
    motor_left(OUTPUT_A),
    motor_right(OUTPUT_D),
    motor_grip(OUTPUT_B),
    
    
    drive_servo_ac(nh, motor_left, motor_right),    
    simple_drive_ac(nh, motor_left, motor_right),
    simple_reverse_ac(nh, motor_left, motor_right),
    driveuntildist_ac(nh, motor_left, motor_right),
    grip_simple_ac(nh, motor_grip)
  {
    r_wheels = 0.14f;    
    motor_left.reset();
    motor_right.reset();
    motor_grip.reset();
    ROS_INFO("Motors are connected");
    
    
  }
  
  void init();
};

#endif
