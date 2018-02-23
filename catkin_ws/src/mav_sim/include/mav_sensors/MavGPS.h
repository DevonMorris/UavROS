#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <std_msgs/Float32.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_params/MavParams.h>

typedef Eigen::Matrix<float, 12, 1> Vector12f;

namespace mav_gps
{

class MavGPS
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Publisher gps_neh_pub_;
  ros::Publisher gps_chi_pub_;
  ros::Publisher gps_vg_pub_;
  ros::Subscriber twist_sub_;

  ros::Time now;

  // callbacks for subs
  void twist_cb_(const geometry_msgs::TwistStampedConstPtr& msg);

  // tf broadcaster
  tf::TransformListener tf_listener_;

  // force and torques
  Eigen::Vector3f vel;

  // params
  mav_params::MavParams params_;

  // random number generator
  std::random_device rd;
  std::mt19937 gen;
  std::normal_distribution<> d;

  // noise params for acc/gyro
  float nu_n;
  float nu_e;
  float nu_h;

  float sig_n;
  float sig_e;
  float sig_h;

  float k_gps;

  float sig_vg;
  float sig_chi;
  
public:
  MavGPS();
  void tick();

}; //end class MavGPS

} //end namespace
