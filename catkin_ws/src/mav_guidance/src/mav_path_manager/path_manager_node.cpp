#include <ros/ros.h>
#include "mav_path_manager/MavPathManager.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_path_manager");

  mav_path_manager::MavPathManager pm;

  ros::Rate r(100);
  ros::spinOnce();
  r.sleep();

  while(ros::ok()){
    pm.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
