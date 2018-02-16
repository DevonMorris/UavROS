#include "mav_controller/MavController.h"

namespace mav_controller
{
  MavController::MavController() :
    nh_(ros::NodeHandle()),
    params_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // Initialize mav_state to trim conditions
    mav_state = Eigen::MatrixXf::Zero(12,1);
    trim_srv_ = nh_.serviceClient<mav_utils::Trim>("/mav/trim"); 

    // publishers and subscribes
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/mav/twist", 5);
    h_sub_ = nh_.subscribe("/mav/h", 5, &MavController::h_cb_, this);
    Va_sub_ = nh_.subscribe("/mav/Va", 5, &MavController::Va_cb_, this);
    Chi_sub_ = nh_.subscribe("/mav/h", 5, &MavController::Chi_cb_, this);
  }

  void MavController::h_cb_(const geometry_msgs::WrenchConstPtr& msg)
  {
    h_c = msg->data;
  }

  void MavController::Va_cb_(const geometry_msgs::WrenchConstPtr& msg)
  {
    Va_c = msg->data;
  }

  void MavController::Chi_cb_(const geometry_msgs::WrenchConstPtr& msg)
  {
    Chi_c = msg->data;
  }

  bool MavController::trim()
  {
    mav_utils::Trim srv;
    nh_.getParam("/mav/Va", srv.request.trims.Va);
    nh_.getParam("/mav/R", srv.request.trims.R);
    nh_.getParam("/mav/gamma", srv.request.trims.gamma);

    if (trim_srv_.call(srv))
    {
      mav_state(3) = srv.response.euler.x;
      mav_state(4) = srv.response.euler.y;
      mav_state(6) = srv.response.vels.linear.x;
      mav_state(7) = srv.response.vels.linear.y;
      mav_state(8) = srv.response.vels.linear.z;
      mav_state(9) = srv.response.vels.angular.x;
      mav_state(10) = srv.response.vels.angular.y;
      mav_state(11) = srv.response.vels.angular.z;

      command.dela = srv.response.commands.dela;
      command.dele = srv.response.commands.dele;
      command.delr = srv.response.commands.delr;
      command.delt = srv.response.commands.delt;
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Could not call trim service");
      return false;
    }
  }

  void MavController::tick()
  {
    // find timestep
    double dt = (ros::Time::now() - now).toSec();
  }

}
