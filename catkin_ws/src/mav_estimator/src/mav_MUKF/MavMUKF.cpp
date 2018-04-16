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
    gamma = .5;
    P_err = .5*Eigen::Matrix<float, 9, 9>::Identity();
    Q_err = .5*Eigen::Matrix<float, 9, 9>::Identity();
    R_err = .01*Eigen::Matrix<float, 5, 5>::Identity();
    resample = true;

    // good numbers according to Craig Bidstrup
    n = 9;
    alpha = 1;
    beta = 2;
    kappa = 0;
    lamb = std::pow(alpha, 2)*(n + kappa) - n;
    mu.w_m  = lamb/(n + lamb);
    mu.w_c  = lamb/(n + lamb) + (1 - std::pow(alpha,2) + beta);
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
    /*
     * This is the kalman update callback
     */
    ROS_WARN_STREAM(mu.dned);
    std::vector<Eigen::Matrix<float, 5, 1>> measurements;

    Eigen::Matrix<float, 5, 1> z_t, z_hat_t;
    z_t << msg->vector.x, msg->vector.y, h_est, Vg_lpf, chi_lpf;
    z_hat_t << 0., 0., 0., 0., 0.;
    

    for (auto&& chi: e_states)
    {
      Eigen::Matrix<float, 5, 1> Z;
      float vg = (mav_n_state.vb + chi.dvb).norm();
      float psi = std::atan2(mav_n_state.vb(1) + chi.dvb(1), 
          mav_n_state.vb(0) + chi.dvb(0));
      Z << chi.dned(0) + mav_n_state.ned(0), chi.dned(1) + mav_n_state.ned(1),
        - mav_n_state.ned(2) - chi.dned(2), vg, psi;
      chi.Z = Z;
      z_hat_t += chi.w_m*Z;
    }

    Eigen::Matrix<float, 5, 5> S_t;
    Eigen::Matrix<float, 9, 5> P_xz;
    S_t = Eigen::Matrix<float, 5, 5>::Zero();
    P_xz = Eigen::Matrix<float, 9, 5>::Zero();

    for (auto&& chi: e_states)
    {
      S_t += chi.w_c*(chi.Z - z_hat_t)*((chi.Z - z_hat_t).transpose());
      P_xz += chi.w_c*((chi - mu).vstate())*((chi.Z - z_hat_t).transpose());
    }
    S_t += R_err;
    Eigen::Matrix<float, 9, 5> K = P_xz*S_t.inverse();

    // consider using joseph form if we go indefinite
    Eigen::Matrix<float, 9, 1> res = K*(z_t - z_hat_t);
    mu.dned += res.block<3,1>(0, 0);
    mu.dvb += res.block<3,1>(3, 0);
    mu.dtheta += res.block<3,1>(6, 0);
    P_err = P_err - K*S_t*K.transpose();

    NState dstate;
    dstate.ned = mu.dned;
    dstate.vb = mu.dvb;
    dstate.Rbv = quat_exp(Eigen::Quaternionf(0., mu.dtheta(0), mu.dtheta(1),
          mu.dtheta(2)));

    //mav_n_state = mav_n_state + dstate;
    mu = EState();
    ROS_WARN_STREAM(mu.dned);

    resample = true;
  }

  void MavMUKF::sample_SigmaX()
  {
    if (!resample)
    {
      return;
    }

    Eigen::LLT<Eigen::Matrix<float,9,9>> Chol(P_err);
    Eigen::Matrix<float,9,9> Sigma = Chol.matrixL();

    e_states.clear();
    e_states.push_back(mu);
    
    for (int i = 0; i<=8; i++)
    {
      // addition sigma
      EState ep;
      ep.dned = mu.dned + gamma*Sigma.block<3,1>(0,i);
      ep.dvb = mu.dvb + gamma*Sigma.block<3,1>(3,i);
      ep.dtheta = mu.dtheta + gamma*Sigma.block<3,1>(6,i);
      ep.dRbv = quat_exp(Eigen::Quaternionf(0, ep.dtheta(0), ep.dtheta(1),
            ep.dtheta(2)));
      ep.w_m = 1/(2*(n + lamb));
      ep.w_c = 1/(2*(n + lamb));
      e_states.push_back(ep);

      //// subtraction sigma
      EState em;
      em.dned = mu.dned - gamma*Sigma.block<3,1>(0,i);
      em.dvb = mu.dvb - gamma*Sigma.block<3,1>(3,i);
      em.dtheta = mu.dtheta - gamma*Sigma.block<3,1>(6,i);
      em.dRbv = quat_exp(Eigen::Quaternionf(0, em.dtheta(0), em.dtheta(1),
            em.dtheta(2)));
      em.w_m = 1/(2*(n + lamb));
      em.w_c = 1/(2*(n + lamb));
      e_states.push_back(em);
    }
    resample = false;
  }

  Eigen::Quaternionf MavMUKF::quat_exp(Eigen::Quaternionf q)
  {
    Eigen::Vector3f q_v;
    q_v << q.x(), q.y(), q.z();
    Eigen::Vector3f q_vexp;
    q_vexp << std::exp(q.w())*q_v.normalized()*std::sin(q_v.norm());
    Eigen::Quaternionf qexp(std::exp(q.w())*std::cos(q_v.norm()),
        q_vexp(0), q_vexp(1), q_vexp(2));
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

  void MavMUKF::f_estate(float dt)
  {
    Eigen::Vector3f g;
    g << 0.0, 0.0, -9.8;

    for(auto&& chi: e_states)
    {
      for(int i = 0; i < 10; i++)
      {
        chi.dned += ((mav_n_state.Rbv*chi.dRbv)*(mav_n_state.vb + chi.dvb) - 
            mav_n_state.Rbv*mav_n_state.vb)*dt/10.;
        chi.dvb += (chi.dvb.cross(gyro) + (mav_n_state.Rbv*chi.dRbv).inverse()*g -
                    mav_n_state.Rbv.inverse()*g)*dt/10.;
        chi.dtheta += (gyro.cross(chi.dtheta))*dt/10.;
      }
      chi.dRbv = quat_exp(Eigen::Quaternionf(0., chi.dtheta(0),
            chi.dtheta(1), chi.dtheta(2)));
    }
    /*
     * Calculate the mean and covariance
     */

    // this will be moved into the callback eventually... maybe
    mu.dned = Eigen::Vector3f::Zero();
    mu.dvb = Eigen::Vector3f::Zero();
    mu.dtheta = Eigen::Vector3f::Zero();
    mu.dRbv = Eigen::Quaternionf::Identity();
    P_err = Eigen::Matrix<float, 9, 9>::Zero();
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
      P_err += chi.w_c*(d.vstate())*(d.vstate().transpose());
    }
    P_err += Q_err;

    mu.dRbv = quat_exp(Eigen::Quaternionf(0., mu.dtheta(0),
          mu.dtheta(1), mu.dtheta(2)));

    // need to think about whether I really need to resample here
  }

  void MavMUKF::tick()
  {
    // predict forward everything!!!!!!
    float dt = (ros::Time::now() - now).toSec();
    now = ros::Time::now();

    // Nominal State
    f_nstate(dt);

    // Error State
    sample_SigmaX();
    f_estate(dt);

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
