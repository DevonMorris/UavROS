#include <ros/ros.h>
#include "tf_viewer/TFViewer.h"

int main(int argc, char **argv){
  // start node
  ros::init(argc, argv, "tf_viewer");

  // instantiate TFViewer object
  tf_viewer::TFViewer frames;

  ros::Rate r(100);
  while(ros::ok()){
    frames.tick();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
