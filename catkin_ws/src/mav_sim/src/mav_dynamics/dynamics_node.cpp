#include <ros/ros.h>
#include "mav_dynamics/MavDynamics.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_dynamics");

  // instantiate TFViewer object
  mav_dynamics::MavDynamics dynam;

  ros::Rate r(100);
  ros::spinOnce();
  r.sleep();

  while(ros::ok()){
    dynam.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
