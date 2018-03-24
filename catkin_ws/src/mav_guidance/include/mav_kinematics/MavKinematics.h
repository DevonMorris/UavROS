#pragma once

#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_params/MavParams.h>

typedef Eigen::Matrix<float, 7, 1> Vector7f;

namespace mav_kinematics
{

class MavKinematics
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Publisher ned_g_pub_;
  ros::Publisher chi_g_pub_;
  ros::Publisher va_g_pub_;

  ros::Subscriber h_sub_;
  ros::Subscriber Va_sub_;
  ros::Subscriber Chi_sub_;
  ros::Subscriber wind_sub_;

  ros::Time now;

  // callbacks for subs
  void h_cb_(const std_msgs::Float32ConstPtr& msg);
  void Va_cb_(const std_msgs::Float32ConstPtr& msg);
  void Chi_cb_(const std_msgs::Float32ConstPtr& msg);
  void wind_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);

  // RK4 and kinematics
  void RK4(double dt);
  // state is n,e,d,ddot,chi,chidot,Va
  Vector7f kinematics(Vector7f state);

  // state of the mav
  Vector7f mav_state;

  Eigen::Vector3f wind;

  // controller params
  float bchidot;
  float bchi;
  float bh;
  float bhdot;
  float bVa;

  // Command variables
  float h_c;
  float Va_c;
  float Chi_c;

  // params
  mav_params::MavParams params_;
  
public:
  MavKinematics();
  void tick();
}; //end class MavKinematics

} //end namespace
