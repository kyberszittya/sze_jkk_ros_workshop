#include "ev3_actions.hpp"


void DriveUntilDistanceActionServer::execute(const ev3_msgs::DriveUntilDistanceGoalConstPtr &goal){
    ev3_msgs::DriveUntilDistanceResult res;
    int current_speed = goal->speed;    
    motor_left.set_speed_sp(current_speed);
    motor_right.set_speed_sp(current_speed);
    motor_right.run_forever();
    motor_left.run_forever();
    std::chrono::system_clock::time_point ref = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point now;
    while (distance > goal->target_distance)
    {
        now = std::chrono::system_clock::now();
        int cnt = std::chrono::duration_cast<std::chrono::milliseconds>(now - ref).count();
        if (cnt > 1000/24){
            driveuntildistance_server.publishFeedback(feedback);
        }
        std::this_thread::yield();        
    }
    motor_left.stop();
    motor_right.stop();
    res.success = true;
    driveuntildistance_server.setSucceeded(res);
}

void DriveUntilDistanceActionServer::cb_ir(const ev3_msgs::SensorIRConstPtr ir)
{
    distance = ir->value;
    feedback.current_distance = distance;
}


void DriveUntilDistanceActionServer::init(){
    sub = nh.subscribe("/ir", 10, &DriveUntilDistanceActionServer::cb_ir, this);
    driveuntildistance_server.start();
    ROS_INFO("Drive until distance action is: UP");    
}