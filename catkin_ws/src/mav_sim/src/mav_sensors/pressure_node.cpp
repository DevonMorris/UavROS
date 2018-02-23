#include <ros/ros.h>
#include "mav_sensors/MavPressure.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_imu");

  // instantiate MavPressure object
  mav_pressure::MavPressure pressure;

  ros::Rate r(100);
  ros::spinOnce();
  r.sleep();

  while(ros::ok()){
    pressure.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
