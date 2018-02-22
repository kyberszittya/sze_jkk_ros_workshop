#include <thread>
#include <chrono>
#include <cmath>
// EV3DEV
#include "ev3dev.h"
// ROS
#include <ros/ros.h>
// Sensor Msgs
#include <ev3_msgs/SensorIR.h>
// Heartbeat
//#include <bondcpp/bond.h>

using namespace ev3dev;

class EV3Sensor 
{
private:
  // ROS
  ros::NodeHandle nh;
  // EV3 Sensor
  infrared_sensor sensor_ir;
  // Sensor threads
  std::thread tx_ir;
  ros::Publisher ir_pub;
  std::chrono::system_clock::time_point start;
  std::chrono::system_clock::time_point ref;
  std::chrono::system_clock::time_point now;
public:
  EV3Sensor(ros::NodeHandle& nh_): nh(nh_), 
    sensor_ir(INPUT_4),
    tx_ir(&EV3Sensor::cbx_ir, this)
    {
        tx_ir.detach();
        ROS_INFO("Started IR thread");
        start = std::chrono::system_clock::now();
  }

  void cbx_ir()
  {
    ir_pub = nh.advertise<ev3_msgs::SensorIR>("/ir",1000);
    ref = std::chrono::system_clock::now();
    while(ros::ok())
    {
      now = std::chrono::system_clock::now();
      int d_sample = std::chrono::duration_cast<std::chrono::milliseconds>(now - ref).count();
      if (sensor_ir.connected() && d_sample > 1000/48)
      {
        ref = std::chrono::system_clock::now();
        int ir_dist = sensor_ir.value();
        ev3_msgs::SensorIR msg;
        
        msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        msg.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start).count() % 1000000000;
        msg.value = ir_dist;
        msg.source_name = "ir0";
        ir_pub.publish(msg);
        ros::spinOnce();        
        std::this_thread::yield();
      }
    }
    ROS_INFO("Stopped IR distance");
    
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ev3_sensor");
  ros::NodeHandle nh;
  /*
  std::string id = "ev3c";
  bond::Bond bond("ev3_component_heartbeat", id);
  ROS_INFO("Starting heartbeat");
  bond.start();
  */
  EV3Sensor ev3sensor(nh);
  
  ros::spin();
  return 0;
}