#pragma once

#include <ros/ros.h>
#include <cmath>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mav_wind
{

  class MavWind
  {
    private:
      // ros node handle
      ros::NodeHandle nh_;

      // ros publishers and subscribers
      ros::Publisher wind_pub_;
      ros::Subscriber twist_sub_;
      
      // tf listener
      tf::TransformListener tf_listener_; 

      // callbacks for subscribers
      void twist_cb_(const geometry_msgs::TwistConstPtr& msg);

      void calcWind();

      Eigen::Vector3f wind;

      Eigen::Vector3f linear;
      Eigen::Vector3f angular;

    public:
      MavWind();
      void tick();
  };
}
