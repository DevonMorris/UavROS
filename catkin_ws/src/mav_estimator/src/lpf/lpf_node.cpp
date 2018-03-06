#include <ros/ros.h>
#include "lpf/LPF.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "sensor_lpf");

  // instantiate TFViewer object
  lpf::LPF sensor_lpf;

  while(ros::ok()){
    ros::spin();
  }
  
  return 0;
}
