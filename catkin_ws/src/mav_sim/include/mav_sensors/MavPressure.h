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

namespace mav_pressure
{

class MavPressure
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Publisher p_static_pub_;
  ros::Publisher p_diff_pub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber wind_sub_;

  ros::Time now;

  // callbacks for subs
  void twist_cb_(const geometry_msgs::TwistStampedConstPtr& msg);
  void wind_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);

  // tf broadcaster
  tf::TransformListener tf_listener_;

  // params
  mav_params::MavParams params_;

  // random number generator
  std::random_device rd;
  std::mt19937 gen;
  std::normal_distribution<> d;

  // wind and body velocities
  Eigen::Vector3f wind;
  Eigen::Vector3f vel;

  // noise params for acc/gyro
  float sig_abs;
  float b_abs;
  float sig_diff;
  float b_diff;
  
public:
  MavPressure();
  void tick();

}; //end class MavPressure

} //end namespace
