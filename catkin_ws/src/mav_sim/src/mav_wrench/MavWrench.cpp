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
  }

  void MavWrench::calcWrench()
  {
    Eigen::Vector3d f_g;
    Eigen::Quaternion<double> R_bv;
    tf::StampedTransform tf_bv;

    try{
      tf_listener_.lookupTransform("vehicle", "base_link", 
          ros::Time(0), tf_bv);
    }
    catch(tf::TransformException &e){
      ROS_ERROR("[/mav_wrench] %s", e.what());
      return;
    }

    /*
     * Gravitational forces
     */

    // Rotate gravity into body frame
    tf::quaternionTFToEigen(tf_bv.getRotation(), R_bv);
    f_g << 0.0, 0.0, params_.m*params_.g;
    f_g = R_bv*f_g;
    auto euler = R_bv.toRotationMatrix().eulerAngles(0,1,2);

    /*
     * Aerodynamic forces and moments
     */

    Eigen::Vector3d Force_aero;
    Eigen::Vector3d moments_aero;

    // Calculate the airspeed
    double V_a2 = linear.squaredNorm();
    double V_a = linear.norm();

    // Calculate angle of attack and rotation from stability to body frame
    // Note: we use the inverse to convert from active rotations to passive rotations
    double alpha = euler(1);
    Eigen::Quaternion<double> R_bs = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitY()).inverse();

    // Calculate longitudinal forces and moments
    double AR = std::pow(params_.b,2)/params_.S;
    double sigma_alpha = sigma(alpha);
    double C_Lflat = 2*sgn(alpha)*std::pow(std::sin(alpha),2)*std::cos(alpha);
    double C_Lalpha = (1.0 - sigma_alpha)*(params_.C_L0 + C_Lalpha*alpha) + sigma_alpha*C_Lflat;
    double C_Dalpha = params_.C_Dp + std::pow(params_.C_L0 + params_.C_Lalpha*alpha, 2) /
      (M_PI*params_.e*AR);
    double F_lift = .5*params_.rho*V_a2*params_.S*(C_Lalpha + 
        params_.C_Lq*(.5*params._c/V_a)*angular.y + params_.C_Ldele*command.dele); 
    double F_drag = .5*params_.rho*V_a2*params_.S*(C_Dalpha + 
        params_.C_Dq*(.5*params._c/V_a)*angular.y + params_.C_Ddele*command.dele); 

    Force_aero << F_drag, 0, F_lift;
    Force_aero = R_bs*Force_aero;

    // Calculate lateral forces and moments


  }

  double MavWrench::sigma(double alpha)
  {
    return (1 + std::exp(-params_.M*(alpha - params_.alpha0)) 
        + std::exp(params_.M*(alpha+params_.alpha0))) /
      ((1+std::exp(-params_.M*(alpha - params_.alpha0)))*
       (1 + std::exp(params_.M*(alpha+params_.alpha0))));
  }

  void MavWrench::command_cb_(const mav_msgs::CommandConstPtr& msg)
  {
    command.dela = msg->dela;
    command.dele = msg->dele;
    command.delr = msg->delr;
    command.thrust = msg->thrust;
  }

  void MavWrench::twist_cb_(const geometry_msgs::TwistConstPtr& msg)
  {
    linear << msg->linear.x, msg->linear.y, msg->linear.z;
    angular << msg->angular.x, msg->angular.y, msg->angular.z;
  }


  void MavWrench::tick()
  {
    geometry_msgs::Wrench msg;
    
    // pack up message and publish
    msg.force.x = force(0);
    msg.force.y = force(1);
    msg.force.z = force(2);
    msg.torque.x = torque(0);
    msg.torque.y = torque(1);
    msg.torque.z = torque(2);
    wrench_pub_.publish(msg);
  }

}
