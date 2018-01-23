#include <ros/ros.h>
#include "mav_wrench/MavWrench.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_wrench");

  // instantiate TFViewer object
  mav_wrench::MavWrench wrench;

  ros::Rate r(100);
  while(ros::ok()){
    wrench.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
