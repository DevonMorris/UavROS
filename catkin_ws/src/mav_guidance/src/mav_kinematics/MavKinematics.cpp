#include "mav_kinematics/MavKinematics.h"

namespace mav_kinematics
{
  MavKinematics::MavKinematics() :
    nh_(ros::NodeHandle()),
    params_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // Initialize mav_state to trim conditions
    mav_state = Eigen::MatrixXf::Zero(7,1);
    mav_state(2) = -200;
    mav_state(6) = 30;

    // publishers and subscribes
    ned_g_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mav/ned_g", 5);
    chi_g_pub_ = nh_.advertise<std_msgs::Float32>("/mav/chi_g", 5);
    va_g_pub_ = nh_.advertise<std_msgs::Float32>("/mav/Va_g", 5);

    h_sub_ = nh_.subscribe("/mav/h_c", 5, &MavKinematics::h_cb_, this);
    Va_sub_ = nh_.subscribe("/mav/Va_c", 5, &MavKinematics::Va_cb_, this);
    Chi_sub_ = nh_.subscribe("/mav/chi_c", 5, &MavKinematics::Chi_cb_, this);
    wind_sub_ = nh_.subscribe("/steady_wind", 5, &MavKinematics::wind_cb_, this);

    wind = Eigen::Vector3f::Zero();

    bchidot = .31;
    bchi = .03;
    bVa = 1;
    bhdot = .3;
    bh = .08;

    h_c = 200;
    Va_c = 30;
  }

  void MavKinematics::h_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    h_c = msg->data;
  }

  void MavKinematics::Va_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    Va_c = msg->data;
  }

  void MavKinematics::Chi_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    Chi_c = msg->data;
  }

  void MavKinematics::wind_cb_(const geometry_msgs::Vector3StampedConstPtr& msg)
  {
    wind << msg->vector.x, msg->vector.y, msg->vector.z;
  }

  void MavKinematics::tick()
  {
    // find timestep
    double dt = (ros::Time::now() - now).toSec();
    now = ros::Time::now();

    // Integrate the kinematics
    RK4(dt);

    // publish ned_g
    geometry_msgs::Vector3Stamped msg_ned;
    msg_ned.vector.x = mav_state(0); msg_ned.vector.y = mav_state(1); msg_ned.vector.z = mav_state(2);
    msg_ned.header.stamp = now;
    ned_g_pub_.publish(msg_ned);

    // publish chi_g
    std_msgs::Float32 msg_chi;
    msg_chi.data = mav_state(4);
    chi_g_pub_.publish(msg_chi);

    // publish Va_g
    std_msgs::Float32 msg_va;
    msg_va.data = mav_state(6);
    va_g_pub_.publish(msg_va);
  }

  void MavKinematics::RK4(double dt)
  {
    Vector7f k_1 = kinematics(mav_state);
    Vector7f k_2 = kinematics(mav_state + (dt/2.0)*k_1);
    Vector7f k_3 = kinematics(mav_state + (dt/2.0)*k_2);
    Vector7f k_4 = kinematics(mav_state + dt*k_3);
    mav_state += (dt/6.0)*(k_1 + 2.0*k_2 + 2.0*k_3 + k_4);
  }

  Vector7f MavKinematics::kinematics(Vector7f state)
  {
    Vector7f state_dot = Eigen::MatrixXf::Zero(7,1);

    //unpack state
    Eigen::Vector3f ned;
    float ddot;
    float chi;
    float chidot;
    float Va;
    float psi;

    ned << state(0), state(1), state(2);
    ddot = state(3); chi = state(4); chidot = state(5); Va = state(6);
    psi = chi - std::asin(-wind(0)*std::sin(chi) + wind(1)*std::cos(chi));

    float ndot = Va*std::cos(psi) + wind(0);
    float edot = Va*std::sin(psi) + wind(1);
    float dddot = bhdot*(-ddot) + bh*(-h_c - ned(2));
    float chiddot = bchidot*(-chidot) + bchi*(Chi_c - chi);
    float Vadot = bVa*(Va_c - Va);


    state_dot << ndot, edot, ddot, dddot, chidot, chiddot, Vadot;

    return state_dot;
  }

}
