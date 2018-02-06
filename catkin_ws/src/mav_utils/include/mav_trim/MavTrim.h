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

struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const{
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

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

  // RK4 and dynamics
  Vector12f dynamics(Vector12f state);

  // Functions to calculate dynamics and forces/moments
  Vector12f dynamics(Vector12f state, Vector6f wrench);
  Vector6f calcWrench(Eigen::Vector3f trim, Eigen::Vector3f abr);

  float sigma(float alpha);

  template <class T>  int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  // inertia matrix
  Eigen::Matrix3f J;
  Eigen::Matrix3f J_inv;

  // params
  mav_params::MavParams params_;
  
public:
  MavTrim();
}; //end class MavTrim

} //end namespace
