#pragma once

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Wrench.h>
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
      ros::Subscriber command_sub_;
      
      // tf listener
      tf::TransformListener tf_listener_; 

      // callbacks for subscribers
      void command_cb_(const mav_msgs::CommandConstPtr& msg);

      Command command;
      void calcWrench();

      Eigen::Vector3d force;
      Eigen::Vector3d torque;

      mav_params::MavParams params_; 

    public:
      MavWrench();
      void tick();
  };
}
