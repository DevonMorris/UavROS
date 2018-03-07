#include "mav_EKF/MavEKF.h"
namespace mav_EKF
{
  MavEKF::MavEKF() :
    nh_(ros::NodeHandle()),
    p_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // publishers and subscribes
    imu_lpf_sub_ = nh_.subscribe("/mav/imu_lpf", 5, &MavEKF::imu_lpf_cb_, this);
    h_lpf_sub_ = nh_.subscribe("/mav/h_lpf", 5, &MavEKF::h_lpf_cb_, this);
    Va_lpf_sub_ = nh_.subscribe("/mav/Va_lpf", 5, &MavEKF::Va_lpf_cb_, this);
    gps_chi_lpf_sub_ = nh_.subscribe("/mav/gps_chi_lpf", 5, &MavEKF::gps_chi_lpf_cb_, this);
    gps_vg_lpf_sub_ = nh_.subscribe("/mav/gps_vg_lpf", 5, &MavEKF::gps_vg_lpf_cb_, this);
    gps_neh_lpf_sub_ = nh_.subscribe("/mav/gps_neh_lpf", 5, &MavEKF::gps_neh_lpf_cb_, this);

    euler_est_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mav/euler_est" ,5);

    P_att = 0.5*Eigen::Matrix2f::Identity();
    att_est = Eigen::Vector2f::Zero(); 
    gyro = Eigen::Vector3f::Zero();
    acc = Eigen::Vector3f::Zero();
    acc(2) = -p_.g;

    Q_att << 1e-7, 0.0,
             0.0, 1e-7;
    R_att << std::pow(.0025*p_.g,2), 0.0, 0.0,
             0.0, std::pow(0.025*p_.g,2), 0.0,
             0.0, 0.0, std::pow(0.025*p_.g,2);
              
    Va_est = 30.;
  }

  // Actually I need to do EKF updates here

  /*
   * Callbacks
   */

  void MavEKF::h_lpf_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    h_est = msg->data;
  }

  void MavEKF::Va_lpf_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    Va_diff = Va_est - msg->data;
    Va_est = msg->data;
  }

  void MavEKF::gps_chi_lpf_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    chi_lpf = msg->data;
  }

  void MavEKF::gps_vg_lpf_cb_(const std_msgs::Float32ConstPtr& msg)
  {
  }

  void MavEKF::imu_lpf_cb_(const sensor_msgs::ImuConstPtr& msg)
  {
    gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    // Kalman update
    Eigen::Vector3f acc_est = h_att(att_est);
    Eigen::Vector3f res = acc - acc_est;
    auto H = dhdx_att(att_est);
    auto S = R_att + H*P_att*H.transpose();
    auto K = P_att*H.transpose()*S.inverse();

    if (acc.norm() < 1.2*p_.g && acc.norm() > .8*p_.g)
    {
      att_est += K*res;
      att_est(0) = wrap(att_est(0));
      att_est(1) = wrap(att_est(1));
      Eigen::Matrix2f I = Eigen::Matrix2f::Identity();
      // Joseph form
      P_att = (I - K*H)*P_att*(I - K*H).transpose() + K*R_att*K.transpose();
    }
  }
  
  void MavEKF::gps_neh_lpf_cb_(const geometry_msgs::Vector3Stamped& msg)
  {
  }

  Eigen::Vector2f MavEKF::f_att(Eigen::Vector2f att)
  {
    float p = gyro(0); float q = gyro(1); float r = gyro(2); 
    float phi = att(0); float theta = att(1);

    Eigen::Vector2f fx;
    fx << p + q*std::sin(phi)*std::tan(theta) + r*std::cos(phi)*std::tan(theta),
          q*std::cos(phi) - r*std::sin(phi);
    return fx;
  }

  Eigen::Vector3f MavEKF::h_att(Eigen::Vector2f att)
  {
    float p = gyro(0); float q = gyro(1); float r = gyro(2); 
    float phi = att(0); float theta = att(1);
  
    Eigen::Vector3f hx;
    hx << q*Va_est*std::sin(theta) + p_.g*std::sin(theta),
          r*Va_est*std::cos(theta) - p*Va_est*std::sin(theta) - p_.g*std::cos(theta)*std::sin(phi),
          -q*Va_est*std::cos(theta) - p_.g*std::cos(theta)*std::cos(phi);

    return hx;
  }

  Eigen::Matrix2f MavEKF::dP_att(Eigen::Matrix2f P, Eigen::Matrix2f A)
  {
    Eigen::Matrix2f dP = A*P + P*A.transpose() + Q_att;
    return dP;
  }

  Eigen::Matrix2f MavEKF::dfdx_att(Eigen::Vector2f att)
  {
    float p = gyro(0); float q = gyro(1); float r = gyro(2); 
    float phi = att(0); float theta = att(1);

    Eigen::Matrix2f dfdx;
    dfdx << q*std::cos(phi)*std::tan(theta) - r*std::sin(phi)*std::tan(theta),
         (q*std::sin(phi) + r*std::cos(phi))/std::pow(std::cos(theta),2),
         -q*std::sin(phi) - r*std::cos(phi),
         0;
    return dfdx;
  }

  Eigen::Matrix<float, 3, 2> MavEKF::dhdx_att(Eigen::Vector2f att)
  {
    float p = gyro(0); float q = gyro(1); float r = gyro(2); 
    float phi = att(0); float theta = att(1);

    Eigen::Matrix<float, 3, 2> dhdx;
    dhdx << 0, q*Va_est*std::cos(theta) + p_.g*std::cos(theta),
         -p_.g*std::cos(phi)*std::cos(theta), -r*Va_est*std::sin(theta) - p*Va_est*std::cos(theta) + p_.g*std::sin(phi)*std::sin(theta),
         p_.g*std::sin(phi)*std::cos(theta), (q*Va_est + p_.g*std::cos(phi))*std::sin(theta);

   return dhdx;
  }

  void MavEKF::RK4_att(float dt)
  {
    auto k1_att = f_att(att_est);
    auto k1_P_att = dP_att(P_att, dfdx_att(att_est));

    auto k2_att = f_att(att_est + dt*k1_att/2.);
    auto k2_P_att = dP_att(P_att + dt/2.*k1_P_att, dfdx_att(att_est + dt*k1_att/2.));

    auto k3_att = f_att(att_est + dt*k2_att/2.);
    auto k3_P_att = dP_att(P_att + dt/2.*k2_P_att, dfdx_att(att_est + dt*k2_att/2.));

    auto k4_att = f_att(att_est + dt*k3_att);
    auto k4_P_att = dP_att(P_att + dt*k3_P_att, dfdx_att(att_est + dt*k3_att));

    att_est += dt*(k1_att + 2.*k2_att + 2.*k3_att + k4_att)/6.;
    P_att += dt*(k1_P_att + 2.*k2_P_att + 2.*k3_P_att + k4_P_att)/6.;
  }

  Eigen::Matrix<float, 5, 1> MavEKF::f_gps(Eigen::Matrix<float, 5, 1> gps_est)
  {
    // state
    float pn = gps_est(0); float pe = gps_est(1); float Vg = gps_est(2);
    float chi = gps_est(3); float psi = gps_est(4);

    // input
    float Va = Va_est; float q = gyro(1); float r = gyro(2);
    float phi = att_est(0); float theta = att_est(1);

    float psi_d = q*std::sin(phi)/std::cos(theta) + r*std::cos(phi)/std::cos(theta);
    
    Eigen::Matrix<float, 5, 1> fx;
    fx << Vg*std::cos(chi),
          Vg*std::sin(chi),
          psi_d*Va*std::sin(chi - psi),
          p_.g*std::tan(phi)*std::cos(chi - psi)/Vg,
          q*std::sin(phi)/std::cos(theta) + r*std::cos(phi)/std::cos(theta);

    return fx;
  }

  Eigen::Matrix<float, 4, 1> MavEKF::h_gps(Eigen::Matrix<float, 5, 1> gps_est)
  {
    // state
    float pn = gps_est(0); float pe = gps_est(1); float Vg = gps_est(2);
    float chi = gps_est(3); float psi = gps_est(4);

    Eigen::Matrix<float, 4, 1> hx;
    hx << pn, pe, Vg, chi;

    return hx;
  }

  Eigen::Matrix<float, 5, 5> MavEKF::dP_gps(Eigen::Matrix<float, 5, 5> P, Eigen::Matrix<float, 5, 5> A)
  {
    Eigen::Matrix<float, 5, 5> dP = A*P + P*A.transpose() + Q_gps;
    return dP;
  }

  Eigen::Matrix<float, 5, 5> MavEKF::dfdx_gps(Eigen::Matrix<float, 5, 1> gps_est)
  {
    // state
    float pn = gps_est(0); float pe = gps_est(1); float Vg = gps_est(2);
    float chi = gps_est(3); float psi = gps_est(4);

    // input
    float Va = Va_est; float q = gyro(1); float r = gyro(2);
    float phi = att_est(0); float theta = att_est(1);

    // approximate derivatives
    float wn = Vg*std::cos(chi) - Va*std::cos(psi);
    float we = Vg*std::sin(chi) - Va*std::sin(psi);
    float psi_d = q*std::sin(phi)/std::cos(theta) + r*std::cos(phi)/std::cos(theta);
    float Vg_d = ((Va*std::cos(psi) + wn)*(-Va*psi_d*std::sin(psi)) +
        (Va*std::sin(psi) + we)*(Va*psi_d*std::cos(psi)))/Vg;

    Eigen::Matrix<float, 5, 5> dfdx;
    dfdx << 0., 0., std::cos(chi), -Vg*std::sin(chi), 0.,
            0., 0., std::sin(chi), Vg*std::cos(chi), 0.,
            0., 0., -Vg_d/Vg, psi_d*Va*std::cos(chi - psi), -psi_d*Va*std::cos(chi - psi),
            0., 0., -p_.g*std::tan(phi)*std::cos(chi - psi)/std::pow(Vg, 2), -p_.g*std::tan(phi)*std::sin(chi - psi)/Vg, p_.g*std::tan(phi)*std::sin(chi - psi),
            0., 0., 0., 0., 0.;

    return dfdx;
  }

  Eigen::Matrix<float, 4, 5> MavEKF::dhdx_gps(Eigen::Matrix<float, 5, 1> gps_est)
  {
    Eigen::Matrix<float, 4, 5> dhdx;
    dhdx << 1., 0., 0., 0., 0.,
            0., 1., 0., 0., 0.,
            0., 0., 1., 0., 0.,
            0., 0., 0., 1., 0.;
    return dhdx;
  }

  void MavEKF::RK4_gps(float dt)
  {
    auto k1_gps = f_gps(gps_est);
    auto k1_P_gps = dP_gps(P_gps, dfdx_gps(gps_est));

    auto k2_gps = f_gps(gps_est + dt*k1_gps/2.);
    auto k2_P_gps = dP_gps(P_gps + dt/2.*k1_P_gps, dfdx_gps(gps_est + dt*k1_gps/2.));

    auto k3_gps = f_gps(gps_est + dt*k2_gps/2.);
    auto k3_P_gps = dP_gps(P_gps + dt/2.*k2_P_gps, dfdx_gps(gps_est + dt*k2_gps/2.));

    auto k4_gps = f_gps(gps_est + dt*k3_gps);
    auto k4_P_gps = dP_gps(P_gps + dt*k3_P_gps, dfdx_gps(gps_est + dt*k3_gps));

    gps_est += dt*(k1_gps + 2.*k2_gps + 2.*k3_gps + k4_gps)/6.;
    P_gps += dt*(k1_P_gps + 2.*k2_P_gps + 2.*k3_P_gps + k4_P_gps)/6.;
  }


  void MavEKF::tick()
  {
    // predict forward everything!!!!!!
    float dt = (ros::Time::now() - now).toSec();
    now = ros::Time::now();

    RK4_att(dt);
    RK4_gps(dt);

    geometry_msgs::Vector3Stamped msg_euler;
    msg_euler.header.stamp = now;
    msg_euler.vector.x = att_est(0);
    msg_euler.vector.y = att_est(1);
    msg_euler.vector.z = gps_est(4);
    euler_est_pub_.publish(msg_euler);

  }

}
