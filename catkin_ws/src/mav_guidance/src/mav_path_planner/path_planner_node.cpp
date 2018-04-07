#include <ros/ros.h>
#include "mav_path_planner/MavPathPlanner.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_path_planner");

  mav_path_planner::MavPathPlanner pp;

  ros::Duration(1).sleep();

  ros::Rate r(1);
  ros::spinOnce();
  r.sleep();

  while(pp.planning){
    pp.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
