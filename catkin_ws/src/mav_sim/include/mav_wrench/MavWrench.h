#pragma once

#include <ros/ros.h>
#include <cmath>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mav_msgs/Command.h>

#include <mav_params/MavParams.h>
#include <mav_utils/Trim.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mav_wrench
{
  typedef struct{
    float dela;
    float dele;
    float delr;
    float delt;
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
      ros::Subscriber wind_sub_;
      
      // tf listener
      tf::TransformListener tf_listener_; 

      // callbacks for subscribers
      void command_cb_(const mav_msgs::CommandConstPtr& msg);
      void twist_cb_(const geometry_msgs::TwistStampedConstPtr& msg);
      void wind_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);

      Command command;
      void calcWrench();
      float sigma(float alpha);

      Eigen::Vector3f force;
      Eigen::Vector3f torque;
      Eigen::Vector3f wind;

      Eigen::Vector3f linear;
      Eigen::Vector3f angular;

      mav_params::MavParams params_; 

      template <class T>  int sgn(T val)
      {
        return (T(0) < val) - (val < T(0));
      }

    public:
      MavWrench();
      bool trim();
      void tick();
  };
}
