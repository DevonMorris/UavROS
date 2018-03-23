#include "mav_controller/MavController.h"
namespace mav_controller
{
  MavController::MavController() :
    nh_(ros::NodeHandle()),
    p_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // Initialize mav_state to trim conditions
    mav_state = Eigen::MatrixXf::Zero(12,1);
    trim_srv_ = nh_.serviceClient<mav_utils::Trim>("/mav/trim"); 

    bool closed = false;
    // publishers and subscribes
    if (closed)
    {
      h_sub_ = nh_.subscribe("/mav/h_c", 5, &MavController::h_cb_, this);
      Va_sub_ = nh_.subscribe("/mav/Va_c", 5, &MavController::Va_cb_, this);
      Chi_sub_ = nh_.subscribe("/mav/chi_c", 5, &MavController::Chi_cb_, this);
      chi_est_sub_ = nh_.subscribe("/mav/chi_est", 5, &MavController::chi_est_cb_, this);
      euler_sub_ = nh_.subscribe("/mav/euler_est", 5, &MavController::euler_cb_, this);
      ned_sub_ = nh_.subscribe("/mav/ned_est", 5, &MavController::ned_cb_, this);
      Va_est_sub_ = nh_.subscribe("/mav/Va_lpf", 5, &MavController::va_est_cb_, this);
    }
    else 
    {
      h_sub_ = nh_.subscribe("/mav/h_c", 5, &MavController::h_cb_, this);
      Va_sub_ = nh_.subscribe("/mav/Va_c", 5, &MavController::Va_cb_, this);
      Chi_sub_ = nh_.subscribe("/mav/chi_c", 5, &MavController::Chi_cb_, this);
      twist_sub_ = nh_.subscribe("/mav/twist", 5, &MavController::twist_cb_, this);
      euler_sub_ = nh_.subscribe("/mav/euler", 5, &MavController::euler_cb_, this);
      ned_sub_ = nh_.subscribe("/mav/ned", 5, &MavController::ned_cb_, this);
    }

    command_pub_ = nh_.advertise<mav_msgs::Command>("/mav/command", 5);

    command.dela = 0.0; command.dele = 0.0; command.delr = 0.0; command.delt = 0.0;
    command_trim = command;
    trimmed = false;

    Va_c = p_.Va;
    Chi_c = 0.0;
    h_c = 200;
    h_takeoff = 50;
    h_hold = 50;

    int_h = 0.0;
    int_v = 0.0;
    int_v2 = 0.0;
    int_takeoff = 0.0;
    int_phi = 0.0;
    int_chi = 0.0;
  }

  void MavController::h_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    h_c = msg->data;
  }

  void MavController::Va_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    Va_c = msg->data;
  }

  void MavController::Chi_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    Chi_c = msg->data;
  }

  void MavController::chi_est_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    chi = msg->data;
  }

  void MavController::va_est_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    V_a = msg->data;
  }

  void MavController::twist_cb_(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    mav_state(6) = msg->twist.linear.x; mav_state(7) = msg->twist.linear.y; mav_state(8) = msg->twist.linear.z;
    mav_state(9) = msg->twist.angular.x; mav_state(10) = msg->twist.angular.y; mav_state(11) = msg->twist.angular.z;

    Eigen::Vector3f vel;
    vel << mav_state(6), mav_state(7), mav_state(8);
    V_a = vel.norm();
    float beta = std::atan2(mav_state(7), mav_state(6));
    chi = beta + mav_state(5);
  }

  void MavController::euler_cb_(const geometry_msgs::Vector3StampedConstPtr& msg)
  {
    mav_state(3) = msg->vector.x; mav_state(4) = msg->vector.y; mav_state(5) = msg->vector.z;
  }

  void MavController::ned_cb_(const geometry_msgs::Vector3StampedConstPtr& msg)
  {
    mav_state(0) = msg->vector.x; mav_state(1) = msg->vector.y; mav_state(2) = msg->vector.z;
  }

  bool MavController::trim()
  {
    mav_utils::Trim srv;
    nh_.getParam("/mav/Va", srv.request.trims.Va);
    nh_.getParam("/mav/R", srv.request.trims.R);
    nh_.getParam("/mav/gamma", srv.request.trims.gamma);

    if (trim_srv_.call(srv))
    {
      mav_state(3) = srv.response.euler.x;
      mav_state(4) = srv.response.euler.y;
      mav_state(6) = srv.response.vels.linear.x;
      mav_state(7) = srv.response.vels.linear.y;
      mav_state(8) = srv.response.vels.linear.z;
      mav_state(9) = srv.response.vels.angular.x;
      mav_state(10) = srv.response.vels.angular.y;
      mav_state(11) = srv.response.vels.angular.z;

      command.dela = srv.response.commands.dela;
      command.dele = srv.response.commands.dele;
      command.delr = srv.response.commands.delr;
      command.delt = srv.response.commands.delt;
      command_trim = command;
    }
    else
    {
      ROS_WARN_STREAM("Could not call trim service");
      return false;
    }

    Eigen::Vector3f trim;
    Eigen::Vector3f abr;

    trim << srv.request.trims.Va, srv.request.trims.R, srv.request.trims.gamma;
    abr << 0.0, 0.0, 0.0;

    float theta = mav_state(4);
    float V_a2 = std::pow(trim(0),2);

    // Gamma terms
    float Gamma = p_.Jx*p_.Jz - std::pow(p_.Jxz,2);
    float Gamma1 = p_.Jxz*(p_.Jx - p_.Jy + p_.Jz)/Gamma;
    float Gamma2 = (p_.Jz*(p_.Jz - p_.Jy) + std::pow(p_.Jxz,2))/Gamma;
    float Gamma3 = p_.Jz/Gamma;
    float Gamma4 = p_.Jxz/Gamma;
    float Gamma5 = (p_.Jz - p_.Jx)/p_.Jy;
    float Gamma6 = p_.Jxz/p_.Jy;
    float Gamma7 = ((p_.Jx - p_.Jy)*p_.Jx + std::pow(p_.Jxz,2))/Gamma;
    float Gamma8 = p_.Jx/Gamma;

    // Roll rate and yaw rate coefficients
    float C_p0 = Gamma3*p_.C_l0 + Gamma4*p_.C_n0;
    float C_pbeta = Gamma3*p_.C_lbeta + Gamma4*p_.C_nbeta;
    float C_pp = Gamma3*p_.C_lp + Gamma4*p_.C_np;
    float C_pr = Gamma3*p_.C_lr + Gamma4*p_.C_nr;
    float C_pdela = Gamma3*p_.C_ldela + Gamma4*p_.C_ndela;
    float C_pdelr = Gamma3*p_.C_ldelr + Gamma4*p_.C_ndelr;
    float C_r0 = Gamma4*p_.C_l0 + Gamma8*p_.C_n0;
    float C_rbeta = Gamma4*p_.C_lbeta + Gamma8*p_.C_nbeta;
    float C_rp = Gamma4*p_.C_lp + Gamma8*p_.C_np;
    float C_rr = Gamma4*p_.C_lr + Gamma8*p_.C_nr;
    float C_rdela = Gamma4*p_.C_ldela + Gamma8*p_.C_ndela;
    float C_rdelr = Gamma4*p_.C_ldelr + Gamma8*p_.C_ndelr;

    a_phi1 = -.25*p_.rho*V_a2*p_.b*p_.b*C_pp*p_.b/trim(0);
    a_phi2 = 0.5*p_.rho*V_a2*p_.S*p_.b*C_pdela;
    a_theta1 = -0.25*p_.rho*V_a2*p_.c*p_.S*p_.c*p_.C_mq/(p_.Jy*trim(0));
    a_theta2 = -.5*p_.rho*V_a2*p_.c*p_.S*p_.C_malpha/p_.Jy;
    a_theta3 = .5*p_.rho*V_a2*p_.c*p_.S*p_.C_mdele/p_.Jy;
    a_V1 = p_.rho*trim(0)*p_.S*(p_.C_D0 + p_.C_Dalpha*abr(0) + p_.C_Ddele*command.dele)/p_.m +
    p_.rho*p_.Sprop*p_.C_prop*trim(0)/p_.m;
    a_V2 = p_.rho*p_.Sprop*p_.C_prop*std::pow(p_.k_motor,2)*command.delt/p_.m;
    a_V3 = p_.g*std::cos(theta - abr(0));
    a_beta1 = -0.5*p_.rho*p_.S*trim(0)*p_.C_Ybeta/p_.m;
    a_beta2 = 0.5*p_.rho*p_.S*trim(0)*p_.C_Ydelr/p_.m;

    // calculate gains for pitch altitude hold
    kp_theta = sgn(a_theta3)*.03/.1;
    float wn_theta = std::sqrt(a_theta2 + kp_theta*a_theta3);
    kd_theta = (3*0.9*wn_theta - a_theta1)/a_theta3;
    k_theta_DC = kp_theta*a_theta3/(a_theta2 + kp_theta*a_theta3);
    
    float wn_v2 = wn_theta/40.;
    ki_v2 = -std::pow(wn_v2,2)/(k_theta_DC*p_.g);
    kp_v2 = 0.5*std::abs((a_V1 - 2*.9*wn_v2)/(k_theta_DC*p_.g));

    float wn_v = 2.2/2.0;
    ki_v = std::pow(wn_v,2)/a_V2; 
    kp_v = 0.5*std::abs((2.0*.8*wn_v - a_V1)/a_V2);

    float wn_h = wn_theta/40.;
    ki_h = std::pow(wn_h,2)/(p_.Va*k_theta_DC);
    kp_h = 2*.9*wn_h/(p_.Va*k_theta_DC);
    kp_h = .02;
    ki_h = .01;

    kp_phi = sgn(a_phi2)*1.0/3.0;
    float wn_phi = std::sqrt(kp_phi*a_phi2);
    kd_phi = (2*2*wn_phi - a_phi1)/a_phi2 + .5;
    ki_phi = 0.01;
    
    float wn_chi = wn_phi/8.0;
    kp_chi = .5*.8*wn_chi*p_.Va/p_.g;
    ki_chi = std::pow(wn_chi,2)*p_.Va/p_.g;

    trimmed = true;

    return true;
  }

  void MavController::tick()
  {
    // find current state of mav
    compute_control();

    // pack up the message and publish
    mav_msgs::Command msg;
    msg.dela = command.dela; msg.dele = command.dele; msg.delr = command.delr; msg.delt = command.delt;
    command_pub_.publish(msg);
  }

  void MavController::compute_control()
  {
    // find timestep
    double dt = (ros::Time::now() - now).toSec();
    now = ros::Time::now();

    /*
     * longitudinal control
     */
    Eigen::Vector3f vel;
    vel << mav_state(6), mav_state(7), mav_state(8);

    float h = -mav_state(2);

    float theta_c;

    // above altitude hold
    //if (h > h_c + h_hold)
    //{
    //  command.delt = 0.0;
    //  command.dele = kp_v2*(Va_c - V_a);
    //  command.dele = saturate(command.dele, -.1, .1);
    //}
    // altitude hold zone
    //else if (h > h_takeoff)
    if ( h > h_takeoff)
    {
      // elevator control
      float e_h = h_c - h;
      e_h = saturate(e_h, -10, 30);
      theta_c = kp_h*(e_h);
      e_h = saturate(e_h, -0.5, 0.5);
      int_h += (e_h)*dt;
      theta_c += ki_h*int_h;
      command.dele = kp_theta*(theta_c - mav_state(4)) - kd_theta*mav_state(10);

      // thrust control
      int_v += (Va_c - V_a)*dt;
      if ((Va_c - V_a) < 0)
        command.delt = command_trim.delt + ki_v*int_v;
      else
        command.delt = command_trim.delt + kp_v*(Va_c - V_a) + ki_v*int_v;
    }
    // takeoff zone
    else
    {
      command.delt = .6;
      theta_c = .2;
      command.dele = kp_theta*(theta_c - mav_state(4)) - kd_theta*mav_state(10);
    }

    /*
     * lateral control
     */

    if (h > h_takeoff)
    {
      // int_chi += dt*(Chi_c - chi);
      float e_chi = 0.0;
      float e_phi = 0.0;
      // wrap chi
      e_chi = Chi_c - chi;
      while (e_chi > (M_PI))
      {
        e_chi = e_chi - 2*M_PI;
      }
      while (e_chi < (-M_PI))
      {
        e_chi = e_chi + 2*M_PI;
      }
      e_chi = saturate(e_chi, -.2, .2); 

      float phi_c = kp_chi*(e_chi) + ki_chi*int_chi;
      phi_c = saturate(phi_c, -.5, .5);

      e_phi = phi_c - mav_state(3);
      // wrap phi
      while (e_phi > M_PI)
      {
        e_phi = e_phi - 2*M_PI;
      }
      while (e_phi < -M_PI)
      {
        e_phi = e_phi + 2*M_PI;
      }
      e_phi = saturate(e_phi, -.3, .3); 

      command.dela = kp_phi*(e_phi) - ki_phi*int_phi - kd_phi*mav_state(9);
    }
    else
    {
      command.dela = 0.0;
    }

    command.dele = saturate(command.dele, -.2, .2);
    command.dela = saturate(command.dela, -.3, .3);
    
    command.delr = 0.0; 
  }

}
