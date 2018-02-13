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
public:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  Eigen::Vector3f trim;

  MavTrim();

private:
  // Ros services
  ros::ServiceServer trim_service_;

  float sigma(float alpha);

  template <class T>  int sgn(T val) 
  {
    return (T(0) < val) - (val < T(0));
  }

  // callbacks for subs
  bool trim_cb_(mav_utils::Trim::Request &req,
      mav_utils::Trim::Response &resp);

  mav_params::MavParams p_;
}; //end class MavTrim

struct TrimFunctor
{
  TrimFunctor(MavTrim* mtrim);

  // Functions to calculate dynamics and forces/moments
  Vector12f dynamics(const double* abr) const;

  float sigma(float alpha) const;

  template <class T>  int sgn(T val) const
  {
    return (T(0) < val) - (val < T(0));
  }

  // Trim conditions
  const Eigen::Vector3f trim;

  // params
  const mav_params::MavParams p_;

  bool operator()(const double* const abr, double* residual) const
  {
    Vector12f xdot, fx;
    xdot << 0.0, 0.0, trim(0)*std::sin(trim(2)), // position derivatives
            0.0, 0.0, 0.0,  // heading derivatives
            0.0, 0.0, trim(0)*std::cos(trim(2))/trim(1), // velocity derivatives
            0.0, 0.0, 0.0; // angular velocity derivatives
    fx = dynamics(abr);
    //ROS_WARN_STREAM("fx " << fx);
    residual[0] = fx(0) - xdot(0);
    residual[1] = fx(1) - xdot(1);
    residual[2] = fx(2) - xdot(2);
    residual[3] = fx(3) - xdot(3);
    residual[4] = fx(4) - xdot(4);
    residual[5] = fx(5) - xdot(5);
    residual[6] = fx(6) - xdot(6);
    residual[7] = fx(7) - xdot(7);
    residual[8] = fx(8) - xdot(8);
    residual[9] = fx(9) - xdot(9);
    residual[10] = fx(10) - xdot(10);
    residual[11] = fx(11) - xdot(11);
    return true;
  }
};

} //end namespace
