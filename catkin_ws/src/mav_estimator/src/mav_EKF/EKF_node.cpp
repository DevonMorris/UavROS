#include <ros/ros.h>
#include "mav_EKF/MavEKF.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_EKF");

  // instantiate TFViewer object
  mav_EKF::MavEKF EKF;

  ros::Rate r(100);
  while(ros::ok()){
    EKF.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
