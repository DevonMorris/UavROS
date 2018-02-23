#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_params/MavParams.h>

typedef Eigen::Matrix<float, 12, 1> Vector12f;

namespace mav_imu
{

class MavIMU
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber wrench_sub_;

  ros::Time now;

  // callbacks for subs
  void twist_cb_(const geometry_msgs::TwistStampedConstPtr& msg);
  void wrench_cb_(const geometry_msgs::WrenchStampedConstPtr& msg);

  // tf broadcaster
  tf::TransformListener tf_listener_;

  // force and torques
  Eigen::Vector3f force;
  Eigen::Vector3f omega;

  // params
  mav_params::MavParams params_;

  // random number generator
  std::random_device rd;
  std::mt19937 gen;
  std::normal_distribution<> d;

  // noise params for acc/gyro
  float sig_ax;
  float sig_ay;
  float sig_az;
  float sig_p;
  float sig_q;
  float sig_r;
  
public:
  MavIMU();
  void tick();

}; //end class MavIMU

} //end namespace
