#include "mav_MUKF/MavMUKF.h"

namespace mav_MUKF
{
  MavMUKF::MavMUKF() :
    nh_(ros::NodeHandle()),
    p_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();
    now_gps = ros::Time::now();

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

    gyro = Eigen::Vector3d::Zero();
    acc = Eigen::Vector3d::Zero();
    acc(2) = -p_.g;

    Va_est = 30.;
    chi_lpf = 0.;
    Vg_lpf = 30.;
    gamma = .5;
    P_err = Eigen::Matrix<double, 9, 9>::Zero();
    // variances position
    P_err(0,0) = 2; P_err(1,1) = 2; P_err(2,2) = 2;
    P_err(3,3) = 1; P_err(4,4) = 1; P_err(5,5) = 1;
    P_err(8,8) = .05; P_err(8,8) = .05; P_err(8,8) = .05; 
    Q_err = .1*Eigen::Matrix<double, 9, 9>::Identity();
    R_err = .01*Eigen::Matrix<double, 8, 8>::Identity();
    resample = true;
    mu = EState({});
    mav_n_state = NState({});
    mav_n_state.ned(2) = -200;
    mav_n_state.vb(0) = 30;
    prev_gps << 0., 0., -200;
    sample_SigmaX();

    // good numbers according to Craig Bidstrup
    n = 9;
    alpha = 1.0;
    beta = 2;
    kappa = .1;
    lamb = std::pow(alpha, 2)*(n + kappa) - n;
    mu.w_m  = lamb/(n + lamb);
    mu.w_c  = lamb/(n + lamb) + (1 - std::pow(alpha,2) + beta);
    initd = true;
  }

  /*
   * Callbacks
   */

  Eigen::Vector3d MavMUKF::QuatToEuler(Eigen::Quaterniond q)
  {
    Eigen::Vector3d euler;
    euler(0) = std::atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(std::pow(q.x(),2) + 
          std::pow(q.y(),2)));
    euler(1) = std::asin(2*(q.w()*q.y() - q.z()*q.x()));
    euler(2) = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(std::pow(q.y(),2) +
          std::pow(q.z(),2)));
    return euler;
  }

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
    /*
     * This is the kalman update callback
     */
    double dt_gps = (ros::Time::now() - now_gps).toSec();
    now_gps = ros::Time::now();

    std::vector<Eigen::Matrix<double, 8, 1>> measurements;

    Eigen::Vector3d vw_gps;
    vw_gps << msg->vector.x, msg->vector.y, -msg->vector.z;
    vw_gps = (vw_gps - prev_gps)/dt_gps;
    prev_gps << msg->vector.x, msg->vector.y, -msg->vector.z;
    mav_n_state.vb = mav_n_state.Rbv.inverse()*vw_gps;

    Eigen::Matrix<double, 8, 1> z_t, z_hat_t;
    z_t << msg->vector.x, msg->vector.y, h_est, vw_gps(0), vw_gps(1), vw_gps(2), chi_lpf, Va_est;
    z_hat_t << 0., 0., 0., 0., 0., 0., 0., 0.;
    

    for (auto&& chi: e_states)
    {
      Eigen::Matrix<double, 8, 1> Z;
      double vg = (mav_n_state.vb + chi.dvb).norm();
      Eigen::Vector3d vw = (mav_n_state.Rbv*chi.dRbv)*(mav_n_state.vb + chi.dvb);
      double psi = std::atan2(vw(1), vw(0));
      Z << chi.dned(0) + mav_n_state.ned(0), chi.dned(1) + mav_n_state.ned(1),
        - mav_n_state.ned(2) - chi.dned(2), vw(0), vw(1), vw(2), psi, mav_n_state.vb(0);
      chi.Z = Z;
      z_hat_t += chi.w_m*Z;
    }

    Eigen::Matrix<double, 8, 8> S_t;
    Eigen::Matrix<double, 9, 8> P_xz;
    S_t = Eigen::Matrix<double, 8, 8>::Zero();
    P_xz = Eigen::Matrix<double, 9, 8>::Zero();

    for (auto&& chi: e_states)
    {
      S_t += chi.w_c*(chi.Z - z_hat_t)*((chi.Z - z_hat_t).transpose());
      P_xz += chi.w_c*((chi - mu).vstate())*((chi.Z - z_hat_t).transpose());
    }

    S_t += R_err;
    Eigen::Matrix<double, 9, 8> K = P_xz*S_t.inverse();

    // consider using joseph form if we go indefinite
    Eigen::Matrix<double, 9, 1> res = K*(z_t - z_hat_t);
    mu.dned += res.block<3,1>(0, 0);
    mu.dvb += res.block<3,1>(3, 0);
    mu.dtheta += res.block<3,1>(6, 0);
    P_err = P_err - K*S_t*K.transpose();

    NState dstate;
    dstate.ned = mu.dned;
    dstate.vb = mu.dvb;
    dstate.Rbv = quat_exp(mu.dtheta);

    mav_n_state = mav_n_state + dstate;
    mu.dned = Eigen::Vector3d::Zero();
    mu.dvb = Eigen::Vector3d::Zero();
    mu.dtheta = Eigen::Vector3d::Zero();
    mu.dRbv = Eigen::Quaterniond::Identity();

    resample = true;
    initd = true;
  }

  void MavMUKF::sample_SigmaX()
  {
    if (!resample)
    {
      return;
    }

    Eigen::LLT<Eigen::Matrix<double,9,9>> Chol(P_err);
    Eigen::Matrix<double,9,9> Sigma = Chol.matrixL();

    e_states.clear();
    e_states.push_back(mu);
    
    for (int i = 0; i<=8; i++)
    {
      // addition sigma
      EState ep({});
      ep.dned = mu.dned + gamma*Sigma.block<3,1>(0,i);
      ep.dvb = mu.dvb + gamma*Sigma.block<3,1>(3,i);
      ep.dtheta = mu.dtheta + gamma*Sigma.block<3,1>(6,i);
      ep.dRbv = quat_exp(ep.dtheta);
      ep.w_m = 1/(2*(n + lamb));
      ep.w_c = 1/(2*(n + lamb));
      e_states.push_back(ep);

      //// subtraction sigma
      EState em({});
      em.dned = mu.dned - gamma*Sigma.block<3,1>(0,i);
      em.dvb = mu.dvb - gamma*Sigma.block<3,1>(3,i);
      em.dtheta = mu.dtheta - gamma*Sigma.block<3,1>(6,i);
      em.dRbv = quat_exp(em.dtheta);
      em.w_m = 1/(2*(n + lamb));
      em.w_c = 1/(2*(n + lamb));
      e_states.push_back(em);
    }
    resample = false;
  }

  Eigen::Quaterniond MavMUKF::quat_exp(Eigen::Vector3d th)
  {
    float eps = 1e-16;
    if (th.norm() < eps)
    {
      return Eigen::Quaterniond::Identity();
    }
    Eigen::Vector3d th_n = th.normalized();
    double cos_nth = std::cos(th.norm()/2.);
    double sin_nth = std::sin(th.norm()/2.);
    Eigen::Quaterniond qexp(cos_nth,
        sin_nth*th_n(0), sin_nth*th_n(1), sin_nth*th_n(2));
    return qexp;
  }

  void MavMUKF::imu_lpf_cb_(const sensor_msgs::ImuConstPtr& msg)
  {
    gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  }

  void MavMUKF::f_nstate(double dt)
  {
    double N = 5.;
    // 10 step euler integration
    // Note since we are using the quaternion exponential, this only needs to be
    // done once for the quaternion
    if (gyro.norm() == 0)
    {
      return;
    }
    Eigen::Vector3d omega_n = gyro.normalized();
    double cos_nomega = std::cos(gyro.norm()*(dt/N)/2.);
    double sin_nomega = std::sin(gyro.norm()*(dt/N)/2.);
    Eigen::Quaterniond exp_omega(cos_nomega, sin_nomega*omega_n(0), 
        sin_nomega*omega_n(1), sin_nomega*omega_n(2));
    Eigen::Vector3d grav; grav << 0., 0., p_.g;
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

  void MavMUKF::f_estate(double dt)
  {
    Eigen::Vector3d g;
    g << 0.0, 0.0, 9.8;

    for(auto&& chi: e_states)
    {
      for(int i = 0; i < 10; i++)
      {
        chi.dned += ((mav_n_state.Rbv*chi.dRbv)*(mav_n_state.vb + chi.dvb) - 
            mav_n_state.Rbv*mav_n_state.vb)*dt/10.;
        chi.dvb += (chi.dvb.cross(gyro) + (mav_n_state.Rbv*chi.dRbv).inverse()*g -
                    mav_n_state.Rbv.inverse()*g)*dt/10.;
        chi.dtheta += -(gyro.cross(chi.dtheta))*dt/10.;
      }
      chi.dRbv = quat_exp(chi.dtheta);
    }
    /*
     * Calculate the mean and covariance
     */

    // this will be moved into the callback eventually... maybe
    mu.dned = Eigen::Vector3d::Zero();
    mu.dvb = Eigen::Vector3d::Zero();
    mu.dtheta = Eigen::Vector3d::Zero();
    mu.dRbv = Eigen::Quaterniond::Identity();
    Eigen::Matrix<double, 9, 9> P_temp = Eigen::Matrix<double, 9, 9>::Zero();
    // calculate mean
    for(auto&& chi: e_states)
    {
      mu.dned += chi.w_m*chi.dned;
      mu.dvb += chi.w_m*chi.dvb;
      mu.dtheta += chi.w_m*chi.dtheta;
    }
    // calculate covariance
    for(auto&& chi: e_states)
    {
      EState d;
      d = chi - mu;
      P_temp += chi.w_c*(d.vstate())*(d.vstate().transpose());
    }
    P_err = P_temp + Q_err;

    mu.dRbv = quat_exp(mu.dtheta);

    resample = true;
    sample_SigmaX();
  }

  void MavMUKF::tick()
  {
    if (!initd)
      return;
    
    // predict forward everything!!!!!!
    double dt = (ros::Time::now() - now).toSec();
    now = ros::Time::now();

    // Nominal State
    f_nstate(dt);

    // Error State
    sample_SigmaX();
    f_estate(dt);

    auto euler = QuatToEuler(mav_n_state.Rbv);

    geometry_msgs::Vector3Stamped msg_euler;
    msg_euler.header.stamp = now;
    msg_euler.vector.x = euler(0);
    msg_euler.vector.y = euler(1);
    msg_euler.vector.z = euler(2);
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

    Eigen::Vector3d v_w = mav_n_state.Rbv*mav_n_state.vb;
    std_msgs::Float32 msg_chi;
    msg_chi.data = std::atan2(v_w(1), v_w(0));
    chi_est_pub_.publish(msg_chi);
  }

}
