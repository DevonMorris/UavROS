#include "lpf/LPF.h"

namespace lpf 
{
  LPF::LPF() :
    nh_(ros::NodeHandle()),
    p_(nh_)
  {
    ros::NodeHandle nh_private("~");

    // initialize member variables to zero
    acc1 = Eigen::Vector3f::Zero();
    gyro1 = Eigen::Vector3f::Zero();
    h1 = 0;
    Va1 = 0;
    gps_Vg1 = 0;
    gps_chi1 = 0;
    
    // publishers and subscribes
    imu_sub_ = nh_.subscribe("/mav/imu", 5, &LPF::imu_cb_, this);
    gps_neh_sub_ = nh_.subscribe("/mav/gps_neh", 5, &LPF::gps_neh_cb_, this);
    gps_chi_sub_ = nh_.subscribe("/mav/gps_chi", 5, &LPF::gps_chi_cb_, this);
    gps_vg_sub_ = nh_.subscribe("/mav/gps_vg", 5, &LPF::gps_vg_cb_, this);
    p_static_sub_ = nh_.subscribe("/mav/p_static", 5, &LPF::p_static_cb_, this);
    p_diff_sub_ = nh_.subscribe("/mav/p_diff", 5, &LPF::p_diff_cb_, this);

    imu_lpf_pub_ = nh_.advertise<sensor_msgs::Imu>("/mav/imu_lpf", 5);
    gps_neh_lpf_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mav/gps_neh_lpf", 5);
    gps_chi_lpf_pub_ = nh_.advertise<std_msgs::Float32>("/mav/gps_chi_lpf", 5);
    gps_vg_lpf_pub_ = nh_.advertise<std_msgs::Float32>("/mav/gps_vg_lpf", 5);
    Va_lpf_pub_ = nh_.advertise<std_msgs::Float32>("/mav/Va_lpf", 5);
    h_lpf_pub_ = nh_.advertise<std_msgs::Float32>("/mav/h_lpf", 5);

    alpha_acc = .9;
    alpha_gyro = .9;
    alpha_gps_neh = .9;
    alpha_h = .9;
    alpha_Va = .9;
    alpha_Vg = .9;
    alpha_chi = .9;
  }

  void LPF::imu_cb_(const sensor_msgs::ImuConstPtr& msg)
  {
    Eigen::Vector3f acc;
    Eigen::Vector3f gyro;

    acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    acc1 = lpf1(acc1, acc, alpha_acc);
    gyro1 = lpf1(gyro1, gyro, alpha_gyro);

    sensor_msgs::Imu msg_o;
    msg_o.linear_acceleration.x = acc1(0);
    msg_o.linear_acceleration.y = acc1(1);
    msg_o.linear_acceleration.z = acc1(2);

    msg_o.angular_velocity.x = gyro1(0);
    msg_o.angular_velocity.y = gyro1(1);
    msg_o.angular_velocity.z = gyro1(2);
    imu_lpf_pub_.publish(msg_o);
  }


  void LPF::gps_neh_cb_(const geometry_msgs::Vector3StampedConstPtr& msg)
  {
    Eigen::Vector3f gps_neh;
    gps_neh << msg->vector.x, msg->vector.y, msg->vector.z;

    gps_neh1 = lpf1(gps_neh1, gps_neh, alpha_gps_neh);
    
    geometry_msgs::Vector3Stamped msg_o;
    msg_o.header = msg->header;
    msg_o.vector.x = gps_neh1(0);
    msg_o.vector.y = gps_neh1(1);
    msg_o.vector.z = gps_neh1(2);

    gps_neh_lpf_pub_.publish(msg_o);
  }

  void LPF::gps_chi_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    float gps_chi = msg->data;
    gps_chi1 = lpf1(gps_chi1, gps_chi, alpha_chi);

    std_msgs::Float32 msg_o;
    msg_o.data = gps_chi1;
    gps_chi_lpf_pub_.publish(msg_o);
  }

  void LPF::gps_vg_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    float gps_Vg = msg->data;
    gps_Vg1 = lpf1(gps_Vg1, gps_Vg, alpha_Vg);

    std_msgs::Float32 msg_o;
    msg_o.data = gps_Vg1;
    gps_vg_lpf_pub_.publish(msg_o);
  }

  void LPF::p_static_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    float h = msg->data/(p_.rho*p_.g);
    h1 = lpf1(h1, h, alpha_h);

    std_msgs::Float32 msg_o;
    msg_o.data = h1;
    h_lpf_pub_.publish(msg_o);
  }

  void LPF::p_diff_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    float Va = std::sqrt(2*msg->data/p_.rho);
    Va1 = lpf1(Va1, Va, alpha_Va);

    std_msgs::Float32 msg_o;
    msg_o.data = Va1;
    Va_lpf_pub_.publish(msg_o);
  }

}
