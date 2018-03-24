#include "mav_sensors/MavGPS.h"

namespace mav_gps
{
  MavGPS::MavGPS() :
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

    // publishers and subscribes
    gps_neh_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mav/gps_neh", 5);
    gps_chi_pub_ = nh_.advertise<std_msgs::Float32>("/mav/gps_chi", 5);
    gps_vg_pub_ = nh_.advertise<std_msgs::Float32>("/mav/gps_vg", 5);

    twist_sub_ = nh_.subscribe("/mav/twist", 5, &MavGPS::twist_cb_, this);

    k_gps = 1.0/11000.0;
    sig_n = .21;
    sig_e = .21;
    sig_h = .4;
  }

  void MavGPS::twist_cb_(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    vel << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
  }

  void MavGPS::tick()
  {
    // find timestep
    double dt = (ros::Time::now() - now).toSec();
    now = ros::Time::now();

    nu_n = std::exp(-k_gps*dt)*nu_n + sig_n*d(gen);
    nu_e = std::exp(-k_gps*dt)*nu_e + sig_e*d(gen);
    nu_h = std::exp(-k_gps*dt)*nu_h + sig_h*d(gen);

    // Get transform from TF tree
    tf::StampedTransform tf_bw;
    tf::StampedTransform tf_vb;
    Eigen::Quaternion<double> R_vb;

    try
    {
      tf_listener_.lookupTransform("world_ned", "base_link", 
          ros::Time(0), tf_bw);
      tf_listener_.lookupTransform("vehicle", "base_link", 
          ros::Time(0), tf_vb);
    }
    catch(tf::TransformException &e)
    {
      return;
    }

    float n = tf_bw.getOrigin().getX();
    float e = tf_bw.getOrigin().getY();
    float h = -tf_bw.getOrigin().getZ();

    // convert rotation into eigen and rotate velocity into correct frame
    tf::quaternionTFToEigen(tf_vb.getRotation(), R_vb);
    vel = R_vb.cast<float>()*vel;

    float Vg = std::sqrt(std::pow(vel(0),2) + std::pow(vel(1),2)) + sig_vg*d(gen);
    float chi = std::atan2(vel(1),vel(0)) + sig_chi*d(gen);

    // pack up msgs and publish
    std_msgs::Float32 msg_gps_vg;
    std_msgs::Float32 msg_gps_chi;

    msg_gps_vg.data = Vg;
    msg_gps_chi.data = chi;

    gps_vg_pub_.publish(msg_gps_vg);
    gps_chi_pub_.publish(msg_gps_chi);

    // pack up neh msg and publish
    geometry_msgs::Vector3Stamped msg_gps_neh;
    msg_gps_neh.header.stamp = now;
    msg_gps_neh.vector.x = n;
    msg_gps_neh.vector.y = e;
    msg_gps_neh.vector.z = h;
    gps_neh_pub_.publish(msg_gps_neh);
  }


}
