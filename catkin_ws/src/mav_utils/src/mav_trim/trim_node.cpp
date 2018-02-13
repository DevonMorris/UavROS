#include <ros/ros.h>
#include "mav_trim/MavTrim.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_trim");

  // Instantiate trim object
  mav_trim::MavTrim trim;

  while(ros::ok()){
    ros::spin();
  }
  
  return 0;
}
