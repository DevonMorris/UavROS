#include "mav_path_follower/MavPathFollower.h"

namespace mav_path_follower
{
  MavPathFollower::MavPathFollower() :
    nh_(ros::NodeHandle()),
    params_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // publishers and subscribes
    h_c_pub_ = nh_.advertise<std_msgs::Float32>("/mav/h_c", 5);
    chi_c_pub_ = nh_.advertise<std_msgs::Float32>("/mav/chi_c", 5);
    phi_ff_pub_ = nh_.advertise<std_msgs::Float32>("/mav/phi_ff", 5);

    path_sub_ = nh_.subscribe("/mav/path", 5, &MavPathFollower::path_cb_, this);
    ned_est_sub_ = nh_.subscribe("/mav/ned_est", 5, &MavPathFollower::ned_est_cb_, this);
    chi_est_sub_ = nh_.subscribe("/mav/chi_est", 5, &MavPathFollower::chi_est_cb_, this);

    p = Eigen::Vector3f::Zero();
    p(2) = -200;

    chi_c = 0;
    chi_est = 0;
    h_c = 200;

    k_path = .05;
    int_rho = 0;
    line = false;
    phi_ff = 0.;
  }

  void MavPathFollower::path_cb_(const mav_msgs::PathConstPtr& msg)
  {
    if (msg->line == true)
    {
      computeLine(msg);
    }
    else
    {
      computeOrbit(msg);
    }
  }

  void MavPathFollower::ned_est_cb_(const geometry_msgs::Vector3StampedConstPtr& msg)
  {
    p << msg->vector.x, msg->vector.y, msg->vector.z;
  }

  void MavPathFollower::chi_est_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    chi_est = msg->data;
  }

  void MavPathFollower::computeLine(const mav_msgs::PathConstPtr& msg)
  {
    Eigen::Vector3f q,r,e,ki,n,s;
    ki << 0, 0, 1;
    q << msg->q.x, msg->q.y, msg->q.z;
    r << msg->r.x, msg->r.y, msg->r.z;

    e = p - r;
    n = q.cross(ki)/(q.cross(ki)).norm();
    s = e - (e.dot(n))*n;
    h_c = -r(2) - std::sqrt(std::pow(s(0),2) + std::pow(s(1),2))*q(2)/
      std::sqrt(std::pow(q(0),2) + std::pow(q(1),2));

    float chi_q = std::atan2(q(1), q(0));
    while (chi_q - chi_est < -M_PI)
      chi_q += 2*M_PI;
    while (chi_q - chi_est > M_PI)
      chi_q -= 2*M_PI;

    float epy = -std::sin(chi_q)*(p(0) - r(0)) + std::cos(chi_q)*(p(1) - r(1));
    float chi_inf = M_PI/4;
    chi_c = chi_q - chi_inf*2*std::atan(k_path*epy)/M_PI;
    phi_ff = 0;

    return;
  }

  void MavPathFollower::computeOrbit(const mav_msgs::PathConstPtr& msg)
  {
    if (msg->rho == 0)
      return;

    Eigen::Vector3f c;
    c << msg->c.x, msg->c.y, msg->c.z;

    h_c = -msg->c.z;
    float d = std::sqrt(std::pow(p(0) - c(0),2) + std::pow(p(1) - c(1),2));
    float phi = std::atan2(p(1) - c(1), p(0) - c(0));

    while (phi - chi_est < -M_PI)
    {
      phi += 2*M_PI;
    }
    while (phi - chi_est > M_PI)
    {
      phi -= 2*M_PI;
    }

    float lam = msg->lamb;
    float rho = msg->rho;
    chi_c = phi + lam*(M_PI/2 + std::atan(.2*(d - rho)/rho));
    phi_ff = lam*std::atan(std::pow(35,2)/(rho*params_.g));

    return;
  }


  void MavPathFollower::tick()
  {

    // publish h_c and chi_c at 100hz
    std_msgs::Float32 msg_chi;
    msg_chi.data = chi_c;
    chi_c_pub_.publish(msg_chi);

    std_msgs::Float32 msg_h;
    msg_h.data = h_c;
    h_c_pub_.publish(msg_h);

    std_msgs::Float32 msg_phi_ff;
    msg_phi_ff.data = phi_ff;
    phi_ff_pub_.publish(msg_phi_ff);

  }

}
