#pragma once

#include <cmath>

#include <ros/ros.h>

#include <vector>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <sensor_msgs/Imu.h>

#include <std_msgs/Float32.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_params/MavParams.h>

typedef Eigen::Matrix<float, 12, 1> Vector12f;

namespace lpf
{

class LPF
{
  typedef struct{
    float dela;
    float dele;
    float delr;
    float delt;
  } Command;

private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_neh_sub_;
  ros::Subscriber gps_chi_sub_;
  ros::Subscriber gps_vg_sub_;
  ros::Subscriber p_static_sub_;
  ros::Subscriber p_diff_sub_;

  ros::Publisher imu_lpf_pub_;
  ros::Publisher gps_neh_lpf_pub_;
  ros::Publisher gps_chi_lpf_pub_;
  ros::Publisher gps_vg_lpf_pub_;
  ros::Publisher Va_lpf_pub_;
  ros::Publisher h_lpf_pub_;

  // callbacks for subscribers
  void imu_cb_(const sensor_msgs::ImuConstPtr& msg);
  void gps_neh_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);
  void gps_chi_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_vg_cb_(const std_msgs::Float32ConstPtr& msg);
  void p_static_cb_(const std_msgs::Float32ConstPtr& msg);
  void p_diff_cb_(const std_msgs::Float32ConstPtr& msg);

  Eigen::Vector3f acc1;
  Eigen::Vector3f gyro1;
  Eigen::Vector3f gps1;
  float h1;
  float Va1;
  float Vg1;
  float chi1;

  // alpha params
  alpha_acc;
  alpha_gyro;
  alpha_gps_neh;
  alpha_h;
  alpha_Va;
  alpha_Vg;
  alpha_chi;

  // sgn function
  mav_params::MavParams p_;

  template <class T, class U>
  T lpf1 (T y, T u, U alpha)
  {
    return alpha*y + (U(1.) - alpha)*u;
  }

  
public:
  LPF();
}; //end class LPF

} //end namespace
