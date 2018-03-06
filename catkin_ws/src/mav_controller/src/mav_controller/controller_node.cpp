#include <ros/ros.h>
#include "mav_controller/MavController.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_controller");

  // instantiate TFViewer object
  mav_controller::MavController controller;

  ros::Rate r(100);
  ros::spinOnce();
  r.sleep();

  while (!controller.trim()){
    ros::spinOnce();
    r.sleep();
  }
  while(ros::ok()){
    controller.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
