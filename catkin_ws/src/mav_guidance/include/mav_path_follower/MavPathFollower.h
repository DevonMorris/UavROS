#pragma once

#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mav_msgs/Path.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_params/MavParams.h>

namespace mav_path_follower
{

class MavPathFollower
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Publisher h_c_pub_;
  ros::Publisher chi_c_pub_;
  ros::Publisher phi_ff_pub_;

  ros::Subscriber path_sub_;
  ros::Subscriber ned_est_sub_;
  ros::Subscriber chi_est_sub_;

  ros::Time now;

  // tf listener
  tf::TransformListener tf_listener_;

  // callbacks for subs
  void path_cb_(const mav_msgs::PathConstPtr& msg);
  void ned_est_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);
  void chi_est_cb_(const std_msgs::Float32ConstPtr& msg);

  void computeLine(const mav_msgs::PathConstPtr& msg);
  void computeOrbit(const mav_msgs::PathConstPtr& msg);

  // Command variables
  Eigen::Vector3f p;
  float h_c;
  float chi_c;
  float chi_est;
  float k_path;

  float phi_ff;
  float int_rho;
  bool line;

  // params
  mav_params::MavParams params_;
  
public:
  MavPathFollower();
  void tick();
}; //end class MavPathFollower

} //end namespace
