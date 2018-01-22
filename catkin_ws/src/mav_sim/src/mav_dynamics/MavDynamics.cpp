#include <mav_dynamics/MavDynamics.h>
#include <iostream>

namespace mav_dynamics
{
  MavDynamics::MavDynamics() :
    nh_(ros::NodeHandle())
  {
      ros::NodeHandle nh_private("~");
      now = ros::Time::now();

      initParams();

      // Initialize state to 0
      mav_state = Eigen::MatrixXd::Zero(12,1);
      // Initialize forces/torques to 0
      force = Eigen::Vector3d::Zero();
      torque = Eigen::Vector3d::Zero();

      // Create inertia matrix
      J << params_.Jx, 0, -params_.Jxz,
           0,     params_.Jy,        0,
           -params_.Jxz, 0, params_.Jz;

      // publishers and subscribes
      input_sub_ = nh_.subscribe("/mav/input", 5, &MavDynamics::ctrl_cb_, this);
  }

  void MavDynamics::initParams()
  {
      // grab params and check if they exist
      if (!nh_.getParam("/mav/m", params_.m) ||
          !nh_.getParam("/mav/Jx", params_.Jx) ||
          !nh_.getParam("/mav/Jy", params_.Jy) ||
          !nh_.getParam("/mav/Jz", params_.Jz) ||
          !nh_.getParam("/mav/Jxz", params_.Jxz) ||
          !nh_.getParam("/mav/S", params_.S) ||
          !nh_.getParam("/mav/b", params_.b) ||
          !nh_.getParam("/mav/c", params_.c) ||
          !nh_.getParam("/mav/Sprop", params_.Sprop) ||
          !nh_.getParam("/mav/rho", params_.rho) ||
          !nh_.getParam("/mav/k_motor", params_.k_motor) ||
          !nh_.getParam("/mav/k_Tp", params_.k_Tp) ||
          !nh_.getParam("/mav/k_Omega", params_.k_Omega) ||
          !nh_.getParam("/mav/e", params_.e) ||
          !nh_.getParam("/mav/C_L0", params_.C_L0) ||
          !nh_.getParam("/mav/C_D0", params_.C_D0) ||
          !nh_.getParam("/mav/C_m0", params_.C_m0) ||
          !nh_.getParam("/mav/C_Lalpha", params_.C_Lalpha) ||
          !nh_.getParam("/mav/C_Dalpha", params_.C_Dalpha) ||
          !nh_.getParam("/mav/C_malpha", params_.C_malpha) ||
          !nh_.getParam("/mav/C_Lq", params_.C_Lq) ||
          !nh_.getParam("/mav/C_Dq", params_.C_Dq) ||
          !nh_.getParam("/mav/C_mq", params_.C_mq) ||
          !nh_.getParam("/mav/C_Ldele", params_.C_Ldele) ||
          !nh_.getParam("/mav/C_Ddele", params_.C_Ddele) ||
          !nh_.getParam("/mav/C_mdele", params_.C_mdele) ||
          !nh_.getParam("/mav/C_prop", params_.C_prop) ||
          !nh_.getParam("/mav/M", params_.M) ||
          !nh_.getParam("/mav/alpha0", params_.alpha0) ||
          !nh_.getParam("/mav/eps", params_.eps) ||
          !nh_.getParam("/mav/C_Dp", params_.C_Dp) ||
          !nh_.getParam("/mav/C_Y0", params_.C_Y0) ||
          !nh_.getParam("/mav/C_l0", params_.C_l0) ||
          !nh_.getParam("/mav/C_n0", params_.C_n0) ||
          !nh_.getParam("/mav/C_Ybeta", params_.C_lbeta) ||
          !nh_.getParam("/mav/C_nbeta", params_.C_nbeta) ||
          !nh_.getParam("/mav/C_Yp", params_.C_Yp) ||
          !nh_.getParam("/mav/C_lp", params_.C_lp) ||
          !nh_.getParam("/mav/C_np", params_.C_np) ||
          !nh_.getParam("/mav/C_Yr", params_.C_Yr) ||
          !nh_.getParam("/mav/C_lr", params_.C_lr) ||
          !nh_.getParam("/mav/C_nr", params_.C_nr) ||
          !nh_.getParam("/mav/C_Ydela", params_.C_Ydela) ||
          !nh_.getParam("/mav/C_ldela", params_.C_ldela) ||
          !nh_.getParam("/mav/C_ndela", params_.C_ndela)) {
        ROS_ERROR("[MavDynamics] params not found on rosparam server");
        ros::shutdown();
      }
  }

  void MavDynamics::ctrl_cb_(const geometry_msgs::WrenchConstPtr& msg)
  {
    force << msg->force.x, msg->force.y, msg->force.z;
    torque << msg->torque.x, msg->torque.y, msg->torque.z;
  }

  void MavDynamics::tick()
  {
    double dt = (now - ros::Time::now()).toSec();
    now = ros::Time::now();

    RK4(dt);

    // create transform
    tf::StampedTransform transform;

    /*
     * Transform from world_ned to body
     */

    transform.setIdentity();
    tf::Quaternion qNED2BODY; qNED2BODY.setRPY(0.0, 0.0, 0.0);
    tf::Vector3 tNED2BODY; tNED2BODY.setValue(mav_state(0), mav_state(1), mav_state(2));

    transform.setRotation(qNED2BODY);
    transform.setOrigin(tNED2BODY);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_ned", "vehicle"));

    tNED2BODY.setValue(0., 0., 0.);
    qNED2BODY.setRPY(0., 0., mav_state(5));
    transform.setRotation(qNED2BODY);
    transform.setOrigin(tNED2BODY);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "vehicle", "vehicle1"));

    tNED2BODY.setValue(0., 0., 0.);
    qNED2BODY.setRPY(mav_state(3), mav_state(4), 0.);
    transform.setRotation(qNED2BODY);
    transform.setOrigin(tNED2BODY);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "vehicle1", "base_link"));

  }

  void MavDynamics::RK4(double dt)
  {
    Vector12d k_1 = dynamics(mav_state);
    Vector12d k_2 = dynamics(mav_state + (dt/2.0)*k_1);
    Vector12d k_3 = dynamics(mav_state + (dt/2.0)*k_2);
    Vector12d k_4 = dynamics(mav_state + dt*k_3);
    mav_state += (dt/6.0)*(k_1 + 2.0*k_2 + 2.0*k_3 + k_4);
  }

  Vector12d MavDynamics::dynamics(Vector12d state)
  {
    Vector12d state_dot = Eigen::MatrixXd::Zero(12,1);

    //state derivatives
    Eigen::Vector3d dpos;
    Eigen::Vector3d datt;
    Eigen::Vector3d dvel;
    Eigen::Vector3d domega;

    //unpack state
    Eigen::Vector3d pos;
    Eigen::Vector3d att;
    Eigen::Vector3d vel;
    Eigen::Vector3d omega;

    pos << state(0), state(1), state(2);
    att << state(3), state(4), state(5);
    vel << state(6), state(7), state(8);
    omega << state(9), state(10), state(11);

    double cphi = std::cos(att(0));
    double sphi = std::sin(att(0));
    double ctheta = std::cos(att(1));
    double stheta = std::sin(att(1));
    double ttheta = std::tan(att(1));
    double cpsi = std::cos(att(2));
    double spsi = std::sin(att(2));


    // Transformation to convert body into vehicle (i.e. NED)
    // note: vehicle to body is given, then we take the inverse (active vs passive)
    Eigen::Quaternion<double> R_vb = Eigen::AngleAxisd(att(0), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(att(1), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(att(2), Eigen::Vector3d::UnitZ()); 
    //Eigen::Matrix3d R_vb;
    //R_vb << ctheta*cpsi, sphi*stheta*cpsi - cphi*spsi, cphi*stheta*cpsi + sphi*spsi,
    //        ctheta*spsi, sphi*stheta*spsi + cphi*cpsi, cphi*stheta*spsi - sphi*cpsi,
    //        -stheta,     sphi*ctheta,     cphi*ctheta;

    // Make matrix for datt
    Eigen::Matrix3d AttD;

    AttD << 1.0, sphi*ttheta, cphi*ttheta,
            0.0, cphi, -sphi,
            0.0, sphi/ctheta, cphi/ctheta;


    // Calculate derivatives
    dpos = R_vb*vel;
    dvel = -omega.cross(vel) + force/params_.m;
    datt = AttD*omega;
    domega = J.inverse()*(-omega.cross(J*omega)+torque);

    state_dot << dpos, datt, dvel, domega;

    return state_dot;
  }

}
