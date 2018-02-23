#include <ros/ros.h>
#include "mav_sensors/MavIMU.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_imu");

  // instantiate MavIMU object
  mav_imu::MavIMU imu;

  ros::Rate r(100);
  ros::spinOnce();
  r.sleep();

  while(ros::ok()){
    imu.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
