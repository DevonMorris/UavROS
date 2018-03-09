#pragma once

#include <ros/ros.h>
#include <cmath>
#include <random>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

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
      ros::Subscriber steady_wind_sub_;
      
      // tf listener
      tf::TransformListener tf_listener_; 

      // callbacks for subscribers
      void steady_wind_cb_(const geometry_msgs::Vector3ConstPtr& msg);

      // random number generator
      std::random_device rd;
      std::mt19937 gen;
      std::normal_distribution<> d;

      void calcWind();

      Eigen::Vector3f wind;
      Eigen::Vector3f wind_b;
      Eigen::Vector3f gust;
      Eigen::Vector3f gust1;
      Eigen::Vector3f gust2;

      Eigen::Vector3f rand;
      Eigen::Vector3f rand1;
      Eigen::Vector3f rand2;

    public:
      MavWind();
      void tick();
  };
}
