#include <ros/ros.h>
#include "mav_path_follower/MavPathFollower.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_path_follower");

  mav_path_follower::MavPathFollower pf;

  ros::Rate r(100);
  ros::spinOnce();
  r.sleep();

  while(ros::ok()){
    pf.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
