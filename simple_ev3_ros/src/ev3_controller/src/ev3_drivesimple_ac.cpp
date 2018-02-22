#include "ev3_actions.hpp"

void DriveSimpleActionServer::executeDriveSimpleCommand(const ev3_msgs::DriveSimpleGoalConstPtr &goal){
    std::chrono::system_clock::time_point start;
    std::chrono::system_clock::time_point now;
    std::cout << goal->speed << '\t' << goal->duration << std::endl;
    ev3_msgs::DriveSimpleResult res;
    start = std::chrono::system_clock::now();
    int current_speed = goal->speed;
    motor_left.set_speed_sp(current_speed);
    motor_right.set_speed_sp(current_speed);
    motor_left.set_time_sp(goal->duration).run_timed();
    motor_right.set_time_sp(goal->duration).run_timed();
    while (motor_left.state().count("running") || motor_right.state().count("running"))
    {
      now = std::chrono::system_clock::now();
      int cnt = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
      feedback.percentage = (cnt*100)/goal->duration;
      std::this_thread::yield();
      drivesimple_server.publishFeedback(feedback);
    }
    
    res.success = true;
    drivesimple_server.setSucceeded(res);
  }

  void DriveSimpleActionServer::init(){
    drivesimple_server.start();
    ROS_INFO("Simple drive action is: UP");
  }