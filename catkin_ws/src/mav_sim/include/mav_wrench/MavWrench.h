#pragma once

#include <ros/ros.h>
#include <cmath>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/Command.h>

#include <mav_params/MavParams.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mav_wrench
{
  typedef struct{
    double dela;
    double dele;
    double delr;
    double thrust;
  } Command;

  class MavWrench
  {
    private:
      // ros node handle
      ros::NodeHandle nh_;

      // ros publishers and subscribers
      ros::Publisher wrench_pub_;
      ros::Subscriber twist_sub_;
      ros::Subscriber command_sub_;
      
      // tf listener
      tf::TransformListener tf_listener_; 

      // callbacks for subscribers
      void command_cb_(const mav_msgs::CommandConstPtr& msg);
      void twist_cb_(const geometry_msgs::TwistConstPtr& msg);

      Command command;
      void calcWrench();
      double sigma(double alpha);

      Eigen::Vector3d force;
      Eigen::Vector3d torque;

      Eigen::Vector3d linear;
      Eigen::Vector3d angular;

      mav_params::MavParams params_; 

      template <class T>  int sgn(T val)
      {
        return (T(0) < val) - (val < T(0));
      }

    public:
      MavWrench();
      void tick();
  };
}
