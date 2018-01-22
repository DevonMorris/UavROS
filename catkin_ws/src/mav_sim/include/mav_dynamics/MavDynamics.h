#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <vector>

#include <mav_msgs/Command.h>

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

  double S;
  double b;
  double c;
  double Sprop;
  double rho;
  double k_motor;
  double k_Tp;
  double k_Omega;
  double e;

  // Longitudinal Coeffs
  double C_L0;
  double C_D0;
  double C_m0;
  double C_Lalpha;
  double C_Dalpha;
  double C_malpha;
  double C_Lq;
  double C_Dq;
  double C_mq;
  double C_Ldele;
  double C_Ddele;
  double C_mdele;
  double C_prop;
  double M;
  double alpha0;
  double eps;
  double C_Dp;

  // Lateral Coeffs
  double C_Y0;
  double C_l0;
  double C_n0;
  double C_Ybeta;
  double C_lbeta;
  double C_nbeta;
  double C_Yp;
  double C_lp;
  double C_np;
  double C_Yr;
  double C_lr;
  double C_nr;
  double C_Ydela;
  double C_ldela;
  double C_ndela;
} parameters_mav;

typedef struct{
  double dela;
  double dele;
  double delr;
  double thrust;
} controls;

class MavDynamics
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Subscriber input_sub_;

  ros::Time now;

  // init for params
  void initParams();

  // callbacks for subs
  void ctrl_cb_(const mav_msgs::CommandConstPtr& msg);

  // RK4 and dynamics
  void RK4(double dt);
  Vector12d dynamics(Vector12d state);

  // tf broadcaster
  tf::TransformBroadcaster tf_br_;

  // state of the mav
  Vector12d mav_state;

  // inputs to the mav
  controls ctrls;

  // inertia matrix
  Eigen::Matrix3d J;

  // params
  parameters_mav params_;
  
public:
  MavDynamics();
  void tick();
}; //end class MavDynamics

} //end namespace
