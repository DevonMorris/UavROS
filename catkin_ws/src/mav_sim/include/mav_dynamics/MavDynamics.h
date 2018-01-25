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

typedef Eigen::Matrix<double, 12, 1> Vector12d;

namespace mav_dynamics
{

class MavDynamics
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Subscriber input_sub_;
  ros::Publisher twist_pub_;

  ros::Time now;

  // callbacks for subs
  void ctrl_cb_(const geometry_msgs::WrenchConstPtr& msg);

  // RK4 and dynamics
  void RK4(double dt);
  Vector12d dynamics(Vector12d state);

  // tf broadcaster
  tf::TransformBroadcaster tf_br_;

  // state of the mav
  Vector12d mav_state;

  // inertia matrix
  Eigen::Matrix3d J;

  // force and torques
  Eigen::Vector3d force;
  Eigen::Vector3d torque;

  // params
  mav_params::MavParams params_;
  
public:
  MavDynamics();
  void tick();
}; //end class MavDynamics

} //end namespace
