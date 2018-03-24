#include <ros/ros.h>
#include "mav_kinematics/MavKinematics.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_kinematics");

  mav_kinematics::MavKinematics kin;

  ros::Rate r(100);
  ros::spinOnce();
  r.sleep();

  while(ros::ok()){
    kin.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
