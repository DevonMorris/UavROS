#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <vector>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <std_msgs/Float32.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_params/MavParams.h>
#include <mav_utils/Trim.h>

#include <mav_msgs/Command.h>

typedef Eigen::Matrix<float, 12, 1> Vector12f;

namespace mav_controller
{

class MavController
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
  ros::Subscriber h_sub_;
  ros::Subscriber Va_sub_;
  ros::Subscriber Chi_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber euler_sub_;
  ros::Subscriber ned_sub_;
  ros::Subscriber Va_est_sub_;
  ros::Subscriber chi_est_sub_;

  ros::Publisher command_pub_;
  ros::ServiceClient trim_srv_;

  // callbacks for subscribers
  void h_cb_(const std_msgs::Float32ConstPtr& msg);
  void Va_cb_(const std_msgs::Float32ConstPtr& msg);
  void Chi_cb_(const std_msgs::Float32ConstPtr& msg);
  void twist_cb_(const geometry_msgs::TwistStampedConstPtr& msg);
  void euler_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);
  void ned_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);
  void chi_est_cb_(const std_msgs::Float32ConstPtr& msg);
  void va_est_cb_(const std_msgs::Float32ConstPtr& msg);

  ros::Time now;

  // tf listener
  tf::TransformListener tf_listener_;

  // params
  float tau; // dirty derivative gain

  float a_phi1;
  float a_phi2;
  float a_theta1;
  float a_theta2;
  float a_theta3;
  float a_V1;
  float a_V2;
  float a_V3;
  float a_beta1;
  float a_beta2;

  float h_takeoff;
  float h_hold;

  // controller params
  float kp_theta;
  float kd_theta;
  float k_theta_DC;

  float kp_phi;
  float kd_phi;
  float ki_phi;
  float int_phi;

  float kp_chi;
  float ki_chi;
  float int_chi;

  float ki_v2;
  float kp_v2;
  float int_v2;

  float ki_v;
  float kp_v;
  float int_v;

  float kp_h;
  float ki_h;
  float int_h;

  float int_takeoff;

  bool trimmed;

  float V_a;
  float chi;


  // command variables
  Command command;
  Command command_trim;
  float h_c;
  float Va_c;
  float Chi_c;

  void compute_control();

  // mav state
  Vector12f mav_state;

  // sgn function
  mav_params::MavParams p_;

  template <class T>  int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  float saturate(float val, float low, float high)
  {
    if(val > high)
      val = high;
    else if (val < low)
      val = low;
    return val;
  }

  
public:
  MavController();
  bool trim();
  void tick();
}; //end class MavController

} //end namespace
