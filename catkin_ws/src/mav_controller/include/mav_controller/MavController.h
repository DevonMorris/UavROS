#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <vector>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

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
  ros::Publisher command_pub_;
  ros::ServiceClient trim_srv_;

  // callbacks for subscribers
  void h_cb_(const std_msgs::Float32ConstPtr& msg);
  void Va_cb_(const std_msgs::Float32ConstPtr& msg);
  void Chi_sub_(const std_msgs::Float32ConstPtr& msg);

  ros::Time now;

  // subscribers

  // tf listener
  tf::TransformListener tf_list_;

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
  float a_beta;

  Command command;

  // command variables
  float h_c;
  float Va_c;
  float Chi_c;

  // inertia matrix
  Eigen::Matrix3f J;
  Eigen::Matrix3f J_inv;

  // mav state
  Vector12f mav_state;

  // force and torques
  Eigen::Vector3f force;
  Eigen::Vector3f torque;

  // params
  mav_params::MavParams params_;
  
public:
  MavController();
  bool trim();
  void tick();
}; //end class MavController

} //end namespace
