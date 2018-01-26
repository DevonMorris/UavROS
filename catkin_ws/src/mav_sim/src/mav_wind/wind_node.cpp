#include <ros/ros.h>
#include <ros/console.h>
#include "mav_wind/MavWind.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "mav_wind");

  // instantiate TFViewer object
  mav_wind::MavWind wind;

  ros::Rate r(100);
  while(ros::ok()){
    wind.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
