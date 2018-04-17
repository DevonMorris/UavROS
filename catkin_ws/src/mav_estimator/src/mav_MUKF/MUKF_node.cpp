#include <ros/ros.h>
#include "mav_MUKF/MavMUKF.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_MUKF");

  // instantiate TFViewer object
  mav_MUKF::MavMUKF MUKF;

  ros::Rate r(50);
  while(ros::ok()){
    MUKF.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
