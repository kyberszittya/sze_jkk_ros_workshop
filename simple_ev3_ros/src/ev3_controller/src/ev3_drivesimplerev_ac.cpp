#include "ev3_actions.hpp"

void DriveSimpleReverseActionServer::executeDriveSimpleReverseCommand(const ev3_msgs::DriveSimpleReverseGoalConstPtr &goal){
    ev3_msgs::DriveSimpleReverseResult res;
    std::chrono::system_clock::time_point start;
    std::chrono::system_clock::time_point now;
    std::cout << goal->speed << '\t' << goal->duration << std::endl;

    start = std::chrono::system_clock::now();
    int current_speed = goal->speed;
    motor_left.set_speed_sp(current_speed);
    motor_right.set_speed_sp(current_speed);
    motor_left.set_time_sp(goal->duration).run_timed();
    motor_right.set_time_sp(goal->duration).run_timed();
    while (motor_left.state().count("running") || motor_right.state().count("running"))
    {
      std::this_thread::yield();
    }
    start = std::chrono::system_clock::now();
    current_speed = -goal->speed;
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
      drivesimplereverse_server.publishFeedback(feedback);
    }
    res.success = true;
    drivesimplereverse_server.setSucceeded(res);
  }

  void DriveSimpleReverseActionServer::init(){
    drivesimplereverse_server.start();
    ROS_INFO("Simple drive-reverse action is: UP");
  }