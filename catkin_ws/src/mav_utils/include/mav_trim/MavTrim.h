#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <vector>

#include <mav_utils/Trim.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_params/MavParams.h>

#include <ceres/ceres.h>
#include <glog/logging.h>

typedef Eigen::Matrix<float, 12, 1> Vector12f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

namespace mav_trim
{

typedef struct{
  float dela;
  float dele;
  float delr;
  float delt;
} Command;

class MavTrim
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;

  // Ros services
  ros::ServiceServer trim_service_;

  // callbacks for subs
  bool trim_cb_(mav_utils::Trim::Request req, mav_utils::Trim::Response resp);

  // Functions to calculate dynamics and forces/moments
  Vector12f dynamics(Vector12f state, Vector6f wrench);
  Vector6f calcWrench(Eigen::Vector3f abr);

  float sigma(float alpha);

  template <class T>  int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  // inertia matrix
  Eigen::Matrix3f J;
  Eigen::Matrix3f J_inv;

  // Trim conditions
  Eigen::Vector3f trim;
  Eigen::Vector3f abr;

  // params
  mav_params::MavParams params_;

  template <typename T>
  bool operator()(const T* const abr, T* residual) 
  {
    Vector6f wrench;
    Vector12f state = Eigen::MatrixXf::Zero(12,1);
    Vector12f state_dot = Eigen::MatrixXf::Zero(12,1);
    Vector12f fx;
    state << 0.0, 0.0, 0.0, // pos
            abr[2], abr[0] + trim(2), 0.0, // heading
            trim(0)*std::cos(abr[0])*std::cos(abr[1]), // u
            trim(0)*std::sin(abr[0]), // v
            trim(0)*std::sin(abr[0])*std::cos(abr[1]), // w
            -trim(0)*std::sin(abr[0] + trim(2))/trim(1), // p
            trim(0)*std::sin(abr[2])*std::cos(abr[0] + trim(2))/trim(1), // q
            trim(0)*std::cos(abr[2])*std::cos(abr[0] + trim(2))/trim(1); // r
            
    wrench = this->calcWrench(abr);
    fx = this->dynamics(state, wrench);
    return true;
  }
  
public:
  MavTrim();
}; //end class MavTrim

struct TrimFunctor{
  TrimFunctor(MavTrim* trim)
    : mtrim_(trim){
  }

  template <typename T>
  bool operator()(const T* const abr, T* residual) const
  {
    Vector6f wrench;
    Vector12f state = Eigen::MatrixXf::Zero(12,1);
    Vector12f state_dot = Eigen::MatrixXf::Zero(12,1);
    Vector12f fx;
    state << 0.0, 0.0, 0.0, // pos
            abr[2], abr[0] + mtrim_->trim(2), 0.0, // heading
            mtrim_->trim(0)*std::cos(abr[0])*std::cos(abr[1]), // u
            mtrim_->trim(0)*std::sin(abr[0]), // v
            mtrim_->trim(0)*std::sin(abr[0])*std::cos(abr[1]), // w
            -mtrim_->trim(0)*std::sin(abr[0] + mtrim_->trim(2))/mtrim_->trim(1), // p
            mtrim_->trim(0)*std::sin(abr[2])*std::cos(abr[0] + mtrim_->trim(2))/mtrim_->trim(1), // q
            mtrim_->trim(0)*std::cos(abr[2])*std::cos(abr[0] + mtrim_->trim(2))/mtrim_->trim(1); // r
            
    wrench = mtrim_->calcWrench(abr);
    fx = mtrim_->dynamics(state, wrench);
    residual = state_dot - fx;
    return true;
  }

private:
    // MavTrim class to access dynamics etc
    MavTrim* mtrim_;
};

} //end namespace
