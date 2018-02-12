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
  // Ros services
  ros::ServiceServer trim_service_;

  // callbacks for subs
  bool trim_cb_(mav_utils::Trim::Request req, mav_utils::Trim::Response resp);

public:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  Eigen::Vector3f trim;

  MavTrim();
}; //end class MavTrim

struct TrimFunctor
{
  TrimFunctor(MavTrim* mtrim);

  // Functions to calculate dynamics and forces/moments
  template <typename T>
  Vector12f dynamics(T abr);

  float sigma(float alpha);

  template <class T>  int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  // Trim conditions
  Eigen::Vector3f trim;

  // params
  mav_params::MavParams p_;

  template <typename T>
  bool operator()(const T* const abr, T* residual) const
  {
    Vector12f xdot, fx;
    xdot << 0.0, 0.0, trim(0)*std::sin(trim(1)), // position derivatives
            0.0, 0.0, 0.0,  // heading derivatives
            0.0, 0.0, trim(0)*std::cos(trim(1))/trim(2), // velocity derivatives
            0.0, 0.0, 0.0; // angular velocity derivatives
    fx = dynamics(abr);
    residual[0] = xdot(0) - fx(0);
    return true;
  }
};

} //end namespace
