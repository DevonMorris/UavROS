#include "mav_wrench/MavWrench.h"

namespace mav_wrench
{
  MavWrench::MavWrench():
    nh_(ros::NodeHandle()),
    params_(nh_)
  {
    // publishers and subscribers
    wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("/mav/wrench", 5);
    command_sub_ = nh_.subscribe("/mav/command", 5, &MavWrench::command_cb_, this);
    twist_sub_ = nh_.subscribe("/mav/twist", 5, &MavWrench::twist_cb_, this);
    wind_sub_ = nh_.subscribe("/mav/wind", 5, &MavWrench::wind_cb_, this);

    // Initialize force and moment
    force = Eigen::Vector3f::Zero();
    torque = Eigen::Vector3f::Zero();
    wind = Eigen::Vector3f::Zero();

    // Initialize linear and angular
    linear = Eigen::Vector3f::Zero();
    angular = Eigen::Vector3f::Zero();

    // Initialize command
    command.dela = 0; command.dele = 0; command.delr = 0; command.delt = 0;
  }

  void MavWrench::calcWrench()
  {
    Eigen::Vector3f Force_g = Eigen::Vector3f::Zero();
    Eigen::Quaternion<double> R_bv;
    tf::StampedTransform tf_bv;

    try
    {
      tf_listener_.lookupTransform("base_link", "vehicle", 
          ros::Time(0), tf_bv);
    }
    catch(tf::TransformException &e)
    {
      return;
    }

    /*
     * Gravitational forces
     */

    // Rotate gravity into body frame
    tf::quaternionTFToEigen(tf_bv.getRotation(), R_bv);
    Force_g << 0.0, 0.0, params_.m*params_.g;
    Force_g = R_bv.cast <float>()*Force_g;

    /*
     * Aerodynamic forces and moments
     */

    Eigen::Vector3f Force_aero = Eigen::Vector3f::Zero();
    Eigen::Vector3f Moment_aero = Eigen::Vector3f::Zero();

    // Calculate the airspeed
    Eigen::Vector3f V_air = linear - wind;
    float V_a2 = V_air.squaredNorm();
    float V_a = V_air.norm();

    // Calculate angle of attack and rotation from stability to body frame
    // Note: we use the inverse to convert from active rotations to passive rotations
    float alpha = std::atan2(V_air(2),V_air(0));
    Eigen::Quaternion<double> R_bs(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitY()));
    R_bs = R_bs.inverse();
    float beta = std::asin(V_air(1)/V_a);

    // Calculate longitudinal forces and moments
    if (V_a < 1 || std::isnan(V_a))
    {
      ROS_WARN_STREAM("Small Airspeed");
      V_a = 1;
    }
    float AR = std::pow(params_.b,2)/params_.S;
    float sigma_alpha = sigma(alpha);
    float C_Lflat = 2*sgn(alpha)*std::pow(std::sin(alpha),2)*std::cos(alpha);
    float C_Lalpha = (1.0 - sigma_alpha)*(params_.C_L0 + params_.C_Lalpha*alpha) + sigma_alpha*C_Lflat;
    //float C_Lalpha = params_.C_L0 + params_.C_Lalpha*alpha;
    float C_Dalpha = params_.C_Dp + std::pow(params_.C_L0 + params_.C_Lalpha*alpha, 2.0) /
      (M_PI*params_.e*AR);
    // float C_Dalpha = params_.C_D0 + params_.C_Dalpha*alpha;

    float F_lift = .5*params_.rho*V_a2*params_.S*(C_Lalpha + 
        params_.C_Lq*(.5*params_.c/V_a)*0*angular(1) + params_.C_Ldele*command.dele); 
    float F_drag = .5*params_.rho*V_a2*params_.S*(C_Dalpha + 
        params_.C_Dq*(.5*params_.c/V_a)*0*angular(1) + params_.C_Ddele*command.dele); 

    Force_aero << -F_drag, 0, -F_lift;
    // coded rotation matrix to check quaternion math
    //Eigen::Matrix3f m;
    //m << std::cos(alpha), 0, -std::sin(alpha),
    //     0, 0, 0,
    //     std::sin(alpha), 0, std::cos(alpha);

    Force_aero = R_bs.cast<float>()*Force_aero;

    Moment_aero(1) = .5*params_.rho*V_a2*params_.S*params_.c*(params_.C_m0 +
        params_.C_malpha*alpha + params_.C_mq*(.5*params_.c/V_a)*angular(1) + 
        params_.C_mdele*command.dele);

  // Calculate lateral forces and moments
    Force_aero(1) = .5*params_.rho*V_a2*params_.S*(params_.C_Y0 +
       params_.C_Ybeta*beta + params_.C_Yp*(.5*params_.b/V_a)*angular(0) + 
       params_.C_Yr*(.5*params_.b/V_a)*angular(2) + params_.C_Ydela*command.dela +
       params_.C_Ydelr*command.delr);

    Moment_aero(0) = .5*params_.rho*V_a2*params_.S*params_.b*(params_.C_l0 +
       params_.C_lbeta*beta + params_.C_lp*(.5*params_.b/V_a)*angular(0) + 
       params_.C_lr*(.5*params_.b/V_a)*angular(2) + params_.C_ldela*command.dela +
       params_.C_ldelr*command.delr);

    Moment_aero(2) = .5*params_.rho*V_a2*params_.S*params_.b*(params_.C_n0 +
       params_.C_nbeta*beta + params_.C_np*(.5*params_.b/V_a)*angular(0) + 
       params_.C_nr*(.5*params_.b/V_a)*angular(2) + params_.C_ndela*command.dela +
       params_.C_ndelr*command.delr);

    /*
     * Propulsion Forces and Moments
     */
    Eigen::Vector3f Force_prop = Eigen::Vector3f::Zero();
    Eigen::Vector3f Moment_prop = Eigen::Vector3f::Zero();

    Force_prop(0) = .5*params_.rho*params_.Sprop*params_.C_prop*
      (std::pow(params_.k_motor*command.delt, 2) - std::pow(V_air(0),2));

    Moment_prop(0) = -params_.k_Tp*std::pow(params_.k_Omega*command.delt,2);

    /*
     * Sum of Forces and Moments
     */
    force = Force_prop + Force_g + Force_aero;
    torque = Moment_prop + Moment_aero;
  }

  float MavWrench::sigma(float alpha)
  {
    return (1.0 + std::exp(-params_.M*(alpha - params_.alpha0)) 
        + std::exp(params_.M*(alpha+params_.alpha0))) /
      ((1.0 + std::exp(-params_.M*(alpha - params_.alpha0)))*
       (1.0 + std::exp(params_.M*(alpha+params_.alpha0))));
  }

  void MavWrench::command_cb_(const mav_msgs::CommandConstPtr& msg)
  {
    command.dela = msg->dela;
    command.dele = msg->dele;
    command.delr = msg->delr;
    command.delt = msg->delt;
  }

  void MavWrench::twist_cb_(const geometry_msgs::TwistConstPtr& msg)
  {
    linear << msg->linear.x, msg->linear.y, msg->linear.z;
    angular << msg->angular.x, msg->angular.y, msg->angular.z;
  }

  void MavWrench::wind_cb_(const geometry_msgs::Vector3ConstPtr& msg)
  {
    wind << msg->x, msg->y, msg->z;
  }

  void MavWrench::tick()
  {
    calcWrench();
    geometry_msgs::Wrench msg;
    
    // pack up message and publish
    msg.force.x = force(0); msg.force.y = force(1); msg.force.z = force(2);
    msg.torque.x = torque(0); msg.torque.y = torque(1); msg.torque.z = torque(2);
    wrench_pub_.publish(msg);
  }

}
