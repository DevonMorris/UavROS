#include <ros/ros.h>
#include <ros/console.h>
#include "mav_wrench/MavWrench.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_wrench");

  // instantiate TFViewer object
  mav_wrench::MavWrench wrench;

  ros::Rate r(100);

  while(!wrench.trim()){
    ros::spinOnce();
    r.sleep();
  }

  while(ros::ok()){
    wrench.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
