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

namespace mav_EKF
{

class MavEKF
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
  ros::Subscriber h_lpf_sub_;
  ros::Subscriber Va_lpf_sub_;
  ros::Subscriber gps_vg_lpf_sub_;
  ros::Subscriber gps_chi_lpf_sub_;
  ros::Subscriber gps_neh_lpf_sub_;
  ros::Subscriber imu_lpf_sub_;

  ros::Publisher ned_est_pub_;
  ros::Publisher euler_est_pub_;
  ros::Publisher twist_est_pub_;

  // callbacks for subscribers
  void h_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void Va_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_vg_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_chi_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_neh_lpf_cb_(const geometry_msgs::Vector3Stamped& msg);
  void imu_lpf_cb_(const sensor_msgs::ImuConstPtr& msg);

  ros::Time now;

  float h_est;
  float Va_est;
  float chi_lpf;

  // mav state for publishing
  Eigen::Vector3f euler_est;

  // states for imu
  Eigen::Vector3f gyro;
  Eigen::Vector3f acc;

  /*
   * Attitude estimation (see section 8.6 uav book)
   */

  // Filter states for att estimation (roll, pitch)
  Eigen::Vector2f att_est;
  Eigen::Matrix2f P_att;

  // Dynamics for att estimation
  Eigen::Vector2f f_att(Eigen::Vector2f att);
  Eigen::Vector3f h_att(Eigen::Vector2f att);
  Eigen::Matrix2f dP_att(Eigen::Matrix2f P, Eigen::Matrix2f A);

  // Jacobians for att estimation
  Eigen::Matrix2f dfdx_att(Eigen::Vector2f att);
  Eigen::Matrix<float, 3, 2> dhdx_att(Eigen::Vector2f att);

  // Covariances for att estimation
  Eigen::Matrix2f Q_att;
  Eigen::Matrix3f R_att;

  // sgn function
  mav_params::MavParams p_;

  // RK4 integration
  void RK4_att(float dt);

public:
  MavEKF();
  bool trim();
  void tick();
}; //end class MavEKF

} //end namespace
