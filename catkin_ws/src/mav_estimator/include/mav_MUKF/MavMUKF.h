#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <vector>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <std_msgs/Float32.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_params/MavParams.h>
#include <mav_utils/Trim.h>


typedef Eigen::Matrix<float, 12, 1> Vector12f;

namespace mav_MUKF
{

struct NState{
  public:
  Eigen::Vector3f ned;
  Eigen::Quaternionf Rbv;
  Eigen::Vector3f vb;

  NState operator+(const NState& b) const
  {
    NState sum;
    sum.ned = this->ned + b.ned;
    sum.Rbv = this->Rbv*b.Rbv;
    sum.vb = this->vb+b.vb;
    return sum;
  }
};


class MavMUKF
{
  private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Subscriber h_lpf_sub_;
  ros::Subscriber Va_lpf_sub_;
  ros::Subscriber gps_vg_lpf_sub_;
  ros::Subscriber gps_chi_lpf_sub_;
  ros::Subscriber gps_neh_lpf_sub_;
  ros::Subscriber imu_lpf_sub_;

  ros::Publisher ned_est_pub_;
  ros::Publisher euler_est_pub_;
  ros::Publisher twist_est_pub_;
  ros::Publisher chi_est_pub_;

  // callbacks for subscribers
  void h_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void Va_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_vg_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_chi_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_neh_lpf_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);
  void imu_lpf_cb_(const sensor_msgs::ImuConstPtr& msg);

  ros::Time now;

  float h_est;
  float Va_est;
  float chi_lpf;
  float Vg_lpf;

  // nominal mav state for integrating 
  NState mav_n_state;

  // states for imu
  Eigen::Vector3f gyro;
  Eigen::Vector3f acc;

  // mav params
  mav_params::MavParams p_;

  // RK4 integration
  void f_nstate(float dt);

  template <class T>
  T wrap(T x)
  {
    while (x > M_PI)
      x -= T(2*M_PI);
    while (x < -M_PI)
      x += T(2*M_PI);
    return x;
  }

public:
  MavMUKF();
  bool trim();
  void tick();
}; //end class MavMUKF

} //end namespace
