#include "mav_wind/MavWind.h"

namespace mav_wind
{
  MavWind::MavWind():
    nh_(ros::NodeHandle())
  {
    // publishers and subscribers
    wind_pub_ = nh_.advertise<geometry_msgs::Vector3>("/mav/wind", 5);
    twist_sub_ = nh_.subscribe("/mav/twist", 5, &MavWind::twist_cb_, this);

    // Initialize wind
    wind << 0.0, 0.0, 0.0;

    // Initialize linear and angular
    linear << 0.0, 0.0, 0.0;
    angular << 0.0, 0.0, 0.0;
  }

  void MavWind::calcWind()
  {
    Eigen::Quaternion<double> R_bv;
    tf::StampedTransform tf_bv;

    try
    {
      tf_listener_.lookupTransform("base_link", "vehicle", 
          ros::Time(0), tf_bv);
    }
    catch(tf::TransformException &e)
    {
      return;
    }

    tf::quaternionTFToEigen(tf_bv.getRotation(), R_bv);

    float V_a = (linear - wind).norm();
  }

  void MavWind::twist_cb_(const geometry_msgs::TwistConstPtr& msg)
  {
    linear << msg->linear.x, msg->linear.y, msg->linear.z;
    angular << msg->angular.x, msg->angular.y, msg->angular.z;
  }

  void MavWind::tick()
  {
    calcWind();
    geometry_msgs::Vector3 msg;
    
    // pack up message and publish
    msg.x = wind(0); msg.y = wind(1); msg.z = wind(2);
    wind_pub_.publish(msg);
  }

}
