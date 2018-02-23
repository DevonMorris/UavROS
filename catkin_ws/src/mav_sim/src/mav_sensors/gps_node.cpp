#include <ros/ros.h>
#include "mav_sensors/MavGPS.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_gps");

  // instantiate MavGPS object
  mav_gps::MavGPS gps;

  ros::Rate r(10);
  ros::spinOnce();
  r.sleep();

  while(ros::ok()){
    gps.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
