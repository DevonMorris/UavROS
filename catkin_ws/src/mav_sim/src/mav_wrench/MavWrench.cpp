#include "mav_wrench/MavWrench.h"

namespace mav_wrench
{
  MavWrench::MavWrench():
    nh_(ros::NodeHandle()),
    params_(nh_)
  {
    // publishers and subscribers
    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("/mav/wrench", 5);
    command_sub_ = nh_.subscribe("/mav/command", 5, &MavWrench::command_cb_, this);


  }

  void MavWrench::calcWrench()
  {
    Eigen::Vector3d f_g;
    Eigen::Quaternion<double> R_bv;
    tf::StampedTransform tf_bv;

    try{
      tf_listener_.lookupTransform("vehicle", "base_link", 
          ros::Time(0), tf_bv);
    }
    catch(tf::TransformException &e){
      ROS_ERROR("[/mav_wrench] %s", e.what());
      return;
    }

    // Rotate gravity into body frame
    tf::quaternionTFToEigen(tf_bv.getRotation(), R_bv);
    f_g << 0.0, 0.0, params_.m*params_.g;
    f_g = R_bv*f_g;


  }

  void MavWrench::command_cb_(const mav_msgs::CommandConstPtr& msg)
  {
    command.dela = msg->dela;
    command.dele = msg->dele;
    command.delr = msg->delr;
    command.thrust = msg->thrust;
  }

  void MavWrench::tick()
  {
    geometry_msgs::Wrench msg;
    
    // pack up message and publish
    msg.force.x = force(0);
    msg.force.y = force(1);
    msg.force.z = force(2);
    msg.torque.x = torque(0);
    msg.torque.y = torque(1);
    msg.torque.z = torque(2);
    wrench_pub_.publish(msg);
  }

}
