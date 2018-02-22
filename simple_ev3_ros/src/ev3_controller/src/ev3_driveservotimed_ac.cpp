#include "ev3_actions.hpp"

void DriveServoTimedActionServer::executeDriveServoTimedCommand(const ev3_msgs::DriveServoTimedGoalConstPtr &goal)
  {
    float ds_left  = (goal->speed)+2*goal->speed*sin(goal->angle);
    float ds_right = (goal->speed)-2*goal->speed*sin(goal->angle);
    std::cout << "Control message: " << ds_left << '\t' << ds_right << " " << goal->speed << " " << goal->angle << " " << goal->time << std::endl;
    if (ds_left < 1000.0){
      motor_left.set_speed_sp(ds_left);
    }else{
      motor_left.set_speed_sp(1000);
    }
    if (ds_right < 1000.0){
      motor_right.set_speed_sp(ds_right);
    }else{
      motor_right.set_speed_sp(1000);
    }
        
    motor_left.set_time_sp(goal->time).run_timed();
    motor_right.set_time_sp(goal->time).run_timed();
    while (motor_left.state().count("running") || motor_right.state().count("running"))
    {
      std::this_thread::yield();
    }
    ev3_msgs::DriveServoTimedResult res;
    res.success = true;
    driveservo_timed_server.setSucceeded(res);
  }

  void DriveServoTimedActionServer::init(){
    driveservo_timed_server.start();
    ROS_INFO("Timed drive action is: UP");    
  }