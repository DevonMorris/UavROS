#pragma once

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Float32.h>

namespace tf_viewer
{

class TFViewer
{

private:
  // Node handles and pub/sub
  ros::NodeHandle nh_;

  ros::Subscriber phi_sub_;
  ros::Subscriber theta_sub_;
  ros::Subscriber psi_sub_;
  ros::Subscriber n_sub_;
  ros::Subscriber e_sub_;
  ros::Subscriber d_sub_;

  void phi_cb_(const std_msgs::Float32ConstPtr& msg);
  void theta_cb_(const std_msgs::Float32ConstPtr& msg);
  void psi_cb_(const std_msgs::Float32ConstPtr& msg);
  void n_cb_(const std_msgs::Float32ConstPtr& msg);
  void e_cb_(const std_msgs::Float32ConstPtr& msg);
  void d_cb_(const std_msgs::Float32ConstPtr& msg);

  float phi_;
  float theta_;
  float psi_;
  float n_;
  float e_;
  float d_;

  // tf broadcaster
  tf::TransformBroadcaster tf_br_;

public:
  TFViewer();
  void tick();
};

}

