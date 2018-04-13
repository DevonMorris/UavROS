#include "mav_MUKF/MavMUKF.h"

namespace mav_MUKF
{
  MavMUKF::MavMUKF() :
    nh_(ros::NodeHandle()),
    p_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // publishers and subscribes
    imu_lpf_sub_ = nh_.subscribe("/mav/imu_lpf", 5, &MavMUKF::imu_lpf_cb_, this);
    h_lpf_sub_ = nh_.subscribe("/mav/h_lpf", 5, &MavMUKF::h_lpf_cb_, this);
    Va_lpf_sub_ = nh_.subscribe("/mav/Va_lpf", 5, &MavMUKF::Va_lpf_cb_, this);
    gps_chi_lpf_sub_ = nh_.subscribe("/mav/gps_chi_lpf", 5, &MavMUKF::gps_chi_lpf_cb_, this);
    gps_vg_lpf_sub_ = nh_.subscribe("/mav/gps_vg_lpf", 5, &MavMUKF::gps_vg_lpf_cb_, this);
    gps_neh_lpf_sub_ = nh_.subscribe("/mav/gps_neh_lpf", 5, &MavMUKF::gps_neh_lpf_cb_, this);

    euler_est_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mav/euler_est" ,5);
    ned_est_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mav/ned_est" ,5);
    chi_est_pub_ = nh_.advertise<std_msgs::Float32>("/mav/chi_est", 5);
    v_est_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mav/vb_est", 5);

    gyro = Eigen::Vector3f::Zero();
    acc = Eigen::Vector3f::Zero();
    acc(2) = -p_.g;

    mav_n_state.ned  << 0., 0., -200;
    mav_n_state.Rbv = Eigen::Quaternion<float>::Identity();
    mav_n_state.vb  << 30., 0., 0.;

    Va_est = 30.;
    chi_lpf = 0.;
    Vg_lpf = 30.;
  }

  /*
   * Callbacks
   */

  void MavMUKF::h_lpf_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    h_est = msg->data;
  }

  void MavMUKF::Va_lpf_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    Va_est = msg->data;
  }

  void MavMUKF::gps_chi_lpf_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    chi_lpf = msg->data;
  }

  void MavMUKF::gps_vg_lpf_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    Vg_lpf = msg->data;
  }

  void MavMUKF::gps_neh_lpf_cb_(const geometry_msgs::Vector3StampedConstPtr& msg)
  {
    return;
  }

  void MavMUKF::imu_lpf_cb_(const sensor_msgs::ImuConstPtr& msg)
  {
    gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  }

  void MavMUKF::f_nstate(float dt)
  {
    float N = 10;
    // 10 step euler integration
    // Note since we are using the quaternion exponential, this only needs to be
    // done once for the quaternion
    if (gyro.norm() == 0)
    {
      return;
    }
    Eigen::Vector3f omega_n = gyro.normalized();
    float cos_nomega = std::cos(gyro.norm()*dt/N/2.);
    float sin_nomega = std::sin(gyro.norm()*dt/N/2.);
    Eigen::Quaternionf exp_omega(cos_nomega, sin_nomega*omega_n(0), 
        sin_nomega*omega_n(1), sin_nomega*omega_n(2));
    Eigen::Vector3f grav; grav << 0., 0., p_.g;
    for (int i = 0; i < N; i++)
    {
      NState dstate;
      dstate.ned = mav_n_state.Rbv*mav_n_state.vb*dt/N;
      dstate.vb = (mav_n_state.vb.cross(gyro) + acc + mav_n_state.Rbv.inverse()*grav)*dt/N;
      dstate.Rbv = exp_omega;

      mav_n_state = mav_n_state + dstate;
    }
    mav_n_state.Rbv = mav_n_state.Rbv.normalized();
  }

  void MavMUKF::tick()
  {
    // predict forward everything!!!!!!
    float dt = (ros::Time::now() - now).toSec();
    now = ros::Time::now();

    f_nstate(dt);

    auto euler = mav_n_state.Rbv.toRotationMatrix().eulerAngles(2, 1, 0);

    geometry_msgs::Vector3Stamped msg_euler;
    msg_euler.header.stamp = now;
    msg_euler.vector.x = euler(2);
    msg_euler.vector.y = euler(1);
    msg_euler.vector.z = euler(0);
    euler_est_pub_.publish(msg_euler);

    geometry_msgs::Vector3Stamped msg_ned;
    msg_ned.header.stamp = now;
    msg_ned.vector.x = mav_n_state.ned(0);
    msg_ned.vector.y = mav_n_state.ned(1);
    msg_ned.vector.z = mav_n_state.ned(2);
    ned_est_pub_.publish(msg_ned);

    geometry_msgs::Vector3Stamped msg_vb;
    msg_vb.header.stamp = now;
    msg_vb.vector.x = mav_n_state.vb(0);
    msg_vb.vector.y = mav_n_state.vb(1);
    msg_vb.vector.z = mav_n_state.vb(2);
    v_est_pub_.publish(msg_vb);

    Eigen::Vector3f v_w = mav_n_state.Rbv*mav_n_state.vb;
    std_msgs::Float32 msg_chi;
    msg_chi.data = std::atan2(v_w(1), v_w(0));
    chi_est_pub_.publish(msg_chi);
  }

}
