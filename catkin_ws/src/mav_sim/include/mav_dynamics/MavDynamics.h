#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <vector>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_params/MavParams.h>
#include <mav_utils/Trim.h>

typedef Eigen::Matrix<float, 12, 1> Vector12f;

namespace mav_dynamics
{

class MavDynamics
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Subscriber input_sub_;
  ros::Publisher twist_pub_;
  ros::ServiceClient trim_srv_;

  ros::Time now;

  // callbacks for subs
  void ctrl_cb_(const geometry_msgs::WrenchConstPtr& msg);

  // RK4 and dynamics
  void RK4(double dt);
  Vector12f dynamics(Vector12f state);

  // tf broadcaster
  tf::TransformBroadcaster tf_br_;

  // state of the mav
  Vector12f mav_state;

  // inertia matrix
  Eigen::Matrix3f J;
  Eigen::Matrix3f J_inv;

  // force and torques
  Eigen::Vector3f force;
  Eigen::Vector3f torque;

  // params
  mav_params::MavParams params_;
  
public:
  MavDynamics();
  void tick();
  bool trim();
}; //end class MavDynamics

} //end namespace
