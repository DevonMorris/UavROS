#pragma once

#include <cmath>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <vector>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <std_msgs/Float32.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

#include <mav_params/MavParams.h>
#include <mav_utils/Trim.h>


typedef Eigen::Matrix<double, 12, 1> Vector12f;

namespace mav_MUKF
{

struct NState{
  public:
    Eigen::Vector3d ned;
    Eigen::Quaterniond Rbv;
    Eigen::Vector3d vb;

    NState()
    {
      ned = Eigen::Vector3d::Zero();
      Rbv = Eigen::Quaterniond::Identity();
      vb = Eigen::Vector3d::Zero();
    }

    NState operator+(const NState& b) const
    {
      NState sum;
      sum.ned = this->ned + b.ned;
      sum.Rbv = this->Rbv*b.Rbv;
      sum.vb = this->vb+b.vb;
      return sum;
    }
};

struct EState{
  public:
    Eigen::Vector3d dned;
    Eigen::Vector3d dvb;
    Eigen::Vector3d dtheta;
    Eigen::Quaterniond dRbv;
    Eigen::Matrix<double, 7, 1> Z;
    double w_m;
    double w_c;

    EState()
    {
      dned = Eigen::Vector3d::Zero();
      dvb = Eigen::Vector3d::Zero();
      dtheta = Eigen::Vector3d::Zero();
      dRbv = Eigen::Quaterniond::Identity();
    }

    EState operator+(const EState& b) const
    {
      EState sum;
      sum.dned = this->dned + b.dned;
      sum.dvb = this->dvb + b.dvb;
      sum.dtheta = this->dtheta + b.dtheta;
      return sum;
    }

    Eigen::Matrix<double, 9, 1> vstate()
    {
      Eigen::Matrix<double, 9, 1> v;
      v << this->dned, this->dvb, this->dtheta;
      return v;
    }

    EState operator-(const EState& b) const
    {
      EState diff;
      diff.dned = this->dned - b.dned;
      diff.dvb = this->dvb - b.dvb;
      diff.dtheta = this->dtheta - b.dtheta;
      return diff;
    }

    // consider moving exp operator into this struct

};


class MavMUKF
{
  private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Subscriber h_lpf_sub_;
  ros::Subscriber Va_lpf_sub_;
  ros::Subscriber gps_vg_lpf_sub_;
  ros::Subscriber gps_chi_lpf_sub_;
  ros::Subscriber gps_neh_lpf_sub_;
  ros::Subscriber imu_lpf_sub_;

  ros::Publisher ned_est_pub_;
  ros::Publisher euler_est_pub_;
  ros::Publisher twist_est_pub_;
  ros::Publisher chi_est_pub_;
  ros::Publisher v_est_pub_;

  // callbacks for subscribers
  void h_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void Va_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_vg_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_chi_lpf_cb_(const std_msgs::Float32ConstPtr& msg);
  void gps_neh_lpf_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);
  void imu_lpf_cb_(const sensor_msgs::ImuConstPtr& msg);

  ros::Time now;
  ros::Time now_gps;

  double h_est;
  double Va_est;
  double chi_lpf;
  double Vg_lpf;

  // nominal mav state for integrating 
  NState mav_n_state;

  // states for imu
  Eigen::Vector3d gyro;
  Eigen::Vector3d acc;

  // state for gps
  Eigen::Vector3d prev_gps;

  // states for ESUKF
  bool resample;
  bool initd;
  double gamma;
  double lamb;
  double alpha;
  double beta;
  double kappa;
  int n;
  EState mu;
  Eigen::Matrix<double, 9, 9> Q_err;
  Eigen::Matrix<double, 7, 7> R_err;
  Eigen::Matrix<double, 9, 9> P_err;

  // mav params
  mav_params::MavParams p_;

  // RK4 integration
  void f_nstate(double dt);
  void f_estate(double dt);

  void sample_SigmaX();

  std::vector<EState> e_states;

  Eigen::Quaterniond quat_exp(Eigen::Vector3d q);

  template <class T>
  T wrap(T x)
  {
    while (x > M_PI)
      x -= T(2*M_PI);
    while (x < -M_PI)
      x += T(2*M_PI);
    return x;
  }

public:
  MavMUKF();
  bool trim();
  void tick();
}; //end class MavMUKF

} //end namespace
