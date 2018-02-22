#include "ev3_actions.hpp"

void GripSimpleActionServer::executeGripSimpleCommand(const ev3_msgs::GripSimpleGoalConstPtr &goal)
{
    ev3_msgs::GripSimpleResult res;
    std::chrono::system_clock::time_point start;
    std::chrono::system_clock::time_point now;

    if (goal->release){
    motor_grip.set_speed_sp(-750);
    motor_grip.set_time_sp(1500).run_timed();
    }
    else{
    motor_grip.set_speed_sp(1000);
    motor_grip.set_time_sp(2500).run_timed();
    }
    feedback.progress = 0;
    start = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point ref = std::chrono::system_clock::now();
    while (motor_grip.state().count("running"))
    {
        now = std::chrono::system_clock::now();
        int cnt = std::chrono::duration_cast<std::chrono::milliseconds>(now - ref).count();
        if (cnt > 1000/92){
            ref = std::chrono::system_clock::now();
            feedback.progress = (cnt*100)/1500;
            gripaction_server.publishFeedback(feedback);
        }
        std::this_thread::yield();        
    }
    res.success = true;
    gripaction_server.setSucceeded(res);
}

void GripSimpleActionServer::init(){
    gripaction_server.start();
    ROS_INFO("Grip action is: UP");    
}