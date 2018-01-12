#pragma once

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <vector>

#include <geometry_msgs/Wrench.h>

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 12, 1> MavState;

namespace mav_dynamics
{

class MavDynamics
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Subscriber input_sub_;

  ros::Time now;

  // callbacks for subs
  void input_cb_(const geometry_msgs::WrenchConstPtr& msg);

  // RK4 and dynamics
  void RK4(double dt, MavState);

  // tf broadcaster
  tf::TransformBroadcaster tf_br_;

  // state of the mav
  MavState mav_state;

  // inputs to the mav
  Eigen::Vector3d force;
  Eigen::Vector3d torque;
  
public:
  MavDynamics();
  void tick();
}; //end class MavDynamics

} //end namespace
