#include "lpf/LPF.h"

namespace lpf 
{
  LPF::LPF() :
    nh_(ros::NodeHandle()),
    p_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // initialize member variables to zero
    acc1 = Eigen::Vector3f::Zero();
    gyro1 = Eigen::Vector3f::Zero();
    h1 = 0;
    Va1 = 0;
    Vg1 = 0;
    chi1 = 0;
    
    // publishers and subscribes
    imu_sub_ = nh_.subscribe("/mav/imu", 5, &LPF::imu_cb_, this);
    gps_neh_sub_ = nh_.subscribe("/mav/gps_neh", 5 &LPF::gps_neh_cb_, this);
    gps_chi_sub_ = nh_.subscribe("/mav/gps_chi", 5 &LPF::gps_chi_cb_, this);
    gps_vg_sub_ = nh_.subscribe("/mav/gps_vg", 5 &LPF::gps_vg_cb_, this);
    p_static_sub_ = nh_.subscribe("/mav/p_static", 5 &LPF::p_static_cb__, this);
    p_diff_sub_ = nh_.subscribe("/mav/p_diff", 5 &LPF::p_diff_cb_, this);

    imu_lpf_pub_ = nh.advertise<sensor_msgs::Imu>("/mav/imu_lpf", 5);
    gps_neh_lpf_pub_ = nh.advertise<geometry_msgs::Vector3f>("/mav/gps_neh_lpf", 5);
    gps_chi_lpf_pub_ = nh.advertise<std_msgs::Float32>("/mav/gps_chi_lpf", 5);
    gps_vg_lpf_pub_ = nh.advertise<std_msgs::Float32>("/mav/gps_vg_lpf", 5);
    Va_lpf_pub_ = nh.advertise<std_msgs::Float32>("/mav/Va_lpf", 5);
    h_lpf_pub_ = nh.advertise<std_msgs::Float32>("/mav/h_lpf", 5);

    alpha_acc = .8;
    alpha_gyro = .8;
    alpha_gps_neh = .8;
    alpha_h = .8;
    alpha_Va = .8;
    alpha_Vg = .8;
    alpha_chi = .8;
  }

  void imu_cb_(const sensor_msgs::ImuConstPtr& msg)
  {
    Eigen::Vector3f acc;
    Eigen::Vector3f gyro;

    acc = msg->linear_acceleration.x, msg->linear_acceleration.y, msg.linear_acceleration.z;
    gyro = msg->angular_velocity.x, msg->angular_velocity.y, msg.angular_velocity.z;

    acc1 = lpf1(acc1, acc, alpha_acc);
    gyro1 = lpf1(gyro1, gyro, alpha_gyro);
  }
  void gps_neh_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);
  void gps_chi_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_vg_cb_(const std_msgs::Float32ConstPtr& msg);
  void p_static_cb_(const std_msgs::Float32ConstPtr& msg);
  void p_diff_cb_(const std_msgs::Float32ConstPtr& msg);

}
