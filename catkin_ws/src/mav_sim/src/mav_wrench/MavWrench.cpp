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

    // Initialize force and moment
    force << 0.0, 0.0, 0.0;
    torque << 0.0, 0.0, 0.0;

    // Initialize linear and angular
    linear << 0.0, 0.0, 0.0;
    angular << 0.0, 0.0, 0.0;

    // Initialize command
    command.dela = 0; command.dele = 0; command.delr = 0; command.delt = 0;
  }

  void MavWrench::calcWrench()
  {
    Eigen::Vector3d Force_g;
    Eigen::Quaternion<double> R_bv;
    tf::StampedTransform tf_bv;

    try
    {
      tf_listener_.lookupTransform("vehicle", "base_link", 
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
    Force_g = R_bv.inverse()*Force_g;
    auto euler = R_bv.toRotationMatrix().eulerAngles(0,1,2);

    /*
     * Aerodynamic forces and moments
     */

    Eigen::Vector3d Force_aero;
    Eigen::Vector3d Moment_aero;

    // Calculate the airspeed
    double V_a2 = linear.squaredNorm();
    double V_a = linear.norm();

    // Calculate angle of attack and rotation from stability to body frame
    // Note: we use the inverse to convert from active rotations to passive rotations
    //ROS_ERROR_STREAM(linear);
    double alpha = std::atan(linear(2)/linear(0));
    Eigen::Quaternion<double> R_bs(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitY()));
    R_bs = R_bs.inverse();

    double beta = 0;

    // Calculate longitudinal forces and moments
    if (V_a < 1e-3 || std::isnan(V_a))
    {
      Force_aero << 0.0, 0.0, 0.0;
      Moment_aero << 0.0, 0.0, 0.0;
    }
    else
    {
      double AR = std::pow(params_.b,2)/params_.S;
      double sigma_alpha = sigma(alpha);
      double C_Lflat = 2*sgn(alpha)*std::pow(std::sin(alpha),2)*std::cos(alpha);
      double C_Lalpha = (1.0 - sigma_alpha)*(params_.C_L0 + C_Lalpha*alpha) + sigma_alpha*C_Lflat;
      double C_Dalpha = params_.C_Dp + std::pow(params_.C_L0 + params_.C_Lalpha*alpha, 2) /
        (M_PI*params_.e*AR);
      double F_lift = .5*params_.rho*V_a2*params_.S*(C_Lalpha + 
          params_.C_Lq*(.5*params_.c/V_a)*angular(1) + params_.C_Ldele*command.dele); 
      double F_drag = .5*params_.rho*V_a2*params_.S*(C_Dalpha + 
          params_.C_Dq*(.5*params_.c/V_a)*angular(1) + params_.C_Ddele*command.dele); 

      Force_aero << -F_drag, 0, -F_lift;
      Force_aero = R_bs*Force_aero;

      Moment_aero(1) = .5*params_.rho*V_a2*params_.S*params_.c*(params_.C_m0 +
          params_.C_malpha*alpha + params_.C_mq*(.5*params_.c/V_a)*angular(1) + 
          params_.C_mdele*command.dele);
    }


    // Calculate lateral forces and moments
    if (V_a < 1e-3 || std::isnan(V_a) || true)
    {
      Force_aero(1) = 0.0;
      Moment_aero(0) = 0.0; Moment_aero(2) = 0.0;
    }
    else
    {
      Force_aero(1) = .5*params_.rho*V_a2*params_.S*(params_.C_Y0 +
         params_.C_Ybeta*beta + params_.C_Yp*(.5*params_.b/V_a)*angular(0) + 
         params_.C_Yr*(.5*params_.b/V_a)*angular(2) + params_.C_Ydela*command.dela +
         params_.C_Ydelr*command.delr);

      Moment_aero(0) = .5*params_.rho*V_a2*params_.S*(params_.C_l0 +
         params_.C_lbeta*beta + params_.C_lp*(.5*params_.b/V_a)*angular(0) + 
         params_.C_lr*(.5*params_.b/V_a)*angular(2) + params_.C_ldela*command.dela +
         params_.C_ldelr*command.delr);

      Moment_aero(2) = .5*params_.rho*V_a2*params_.S*(params_.C_n0 +
         params_.C_nbeta*beta + params_.C_np*(.5*params_.b/V_a)*angular(0) + 
         params_.C_nr*(.5*params_.b/V_a)*angular(2) + params_.C_ndela*command.dela +
         params_.C_ndelr*command.delr);
    }

    /*
     * Propulsion Forces and Moments
     */
    Eigen::Vector3d Force_prop;
    Eigen::Vector3d Moment_prop;

    Force_prop(0) = .5*params_.rho*params_.Sprop*params_.C_prop*
      (std::pow(params_.k_motor*command.delt, 2) - V_a2);

    Moment_prop(0) = -params_.k_Tp*std::pow(params_.k_Omega*command.delt,2);

    /*
     * Sum of Forces and Moments
     */
    force = Force_prop + Force_g + Force_aero;
    torque = Moment_prop;// + Moment_aero;
  }

  double MavWrench::sigma(double alpha)
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
