#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <vector>

#include <geometry_msgs/Wrench.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef Eigen::Matrix<double, 12, 1> Vector12d;

namespace mav_dynamics
{

typedef struct
{
  double m; // mass

  // moments of interia
  double Jx; 
  double Jy;
  double Jz;
  double Jxz;
} parameters_mav;

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
  void RK4(double dt);
  Vector12d dynamics(Vector12d state);

  // tf broadcaster
  tf::TransformBroadcaster tf_br_;

  // state of the mav
  Vector12d mav_state;

  // inputs to the mav
  Eigen::Vector3d force;
  Eigen::Vector3d torque;

  // inertia matrix
  Eigen::Matrix3d J;

  // params
  parameters_mav params_;
  
public:
  MavDynamics();
  void tick();
}; //end class MavDynamics

} //end namespace
