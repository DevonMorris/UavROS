#pragma once

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Pose.h>

namespace tf_frames
{

class TFFrames
{

private:
  // Node handles and pub/sub
  ros::NodeHandle nh_;

  // tf broadcaster
  tf::TransformBroadcaster tf_br_;

public:
  TFFrames();
  void tick();
};

}

