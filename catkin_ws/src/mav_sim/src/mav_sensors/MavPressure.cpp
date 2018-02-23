#include "mav_sensors/MavPressure.h"

namespace mav_pressure
{
  MavPressure::MavPressure() :
    nh_(ros::NodeHandle()),
    params_(nh_),
    rd(),
    gen(rd()),
    d(0,1)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // Initialize forces/angular vels to 0
    vel = Eigen::Vector3f::Zero();
    wind = Eigen::Vector3f::Zero();

    // publishers and subscribes
    p_static_pub_ = nh_.advertise<std_msgs::Float32>("/mav/p_static", 5);
    p_diff_pub_ = nh_.advertise<std_msgs::Float32>("/mav/p_diff", 5);
    twist_sub_ = nh_.subscribe("/mav/twist", 5, &MavPressure::twist_cb_, this);
    wind_sub_ = nh_.subscribe("/mav/wind", 5, &MavPressure::wind_cb_, this);

    // setup std dev for acc
    sig_abs = .01;
    b_abs = .125;
    sig_diff = .002;
    b_diff = .02;
  }

  void MavPressure::wind_cb_(const geometry_msgs::Vector3StampedConstPtr& msg)
  {
    wind << msg->vector.x, msg->vector.y, msg->vector.z;
  }

  void MavPressure::twist_cb_(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    vel << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
  }

  void MavPressure::tick()
  {
    // find timestep
    double dt = (ros::Time::now() - now).toSec();
    now = ros::Time::now();

    // Get transform from TF tree
    Eigen::Vector3f Force_g = Eigen::Vector3f::Zero();
    Eigen::Quaternion<double> R_bv;
    tf::StampedTransform tf_bv;

    try
    {
      tf_listener_.lookupTransform("world_ned", "base_link", 
          ros::Time(0), tf_bv);
    }
    catch(tf::TransformException &e)
    {
      return;
    }

    Eigen::Vector3f V_a;
    V_a = vel - wind;

    float Va2 = V_a.squaredNorm();

    float h = -tf_bv.getOrigin().getZ();
    float p_abs = params_.rho*params_.g*h + b_abs + sig_abs*d(gen);

    float p_diff = .5*params_.rho*Va2 + b_diff + sig_diff*d(gen);

    // pack up messages and publish
    std_msgs::Float32 p_diff_msg; 
    p_diff_msg.data = p_diff;
    p_diff_pub_.publish(p_diff_msg);

    std_msgs::Float32 p_abs_msg; 
    p_abs_msg.data = p_abs;
    p_static_pub_.publish(p_abs_msg);
  }


}
