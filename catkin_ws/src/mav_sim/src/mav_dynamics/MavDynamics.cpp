#include "mav_dynamics/MavDynamics.h"

namespace mav_dynamics
{
  MavDynamics::MavDynamics() :
    nh_(ros::NodeHandle()),
    params_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/mav/twist", 5);

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
    input_sub_ = nh_.subscribe("/mav/wrench", 5, &MavDynamics::ctrl_cb_, this);
  }

  void MavDynamics::ctrl_cb_(const geometry_msgs::WrenchConstPtr& msg)
  {
    force << msg->force.x, msg->force.y, msg->force.z;
    torque << msg->torque.x, msg->torque.y, msg->torque.z;
  }

  void MavDynamics::tick()
  {
    // find timestep
    double dt = (now - ros::Time::now()).toSec();
    now = ros::Time::now();

    // Integrate the dynamics
    RK4(dt);

    // create transform
    tf::StampedTransform transform;

    transform.setIdentity();
    tf::Quaternion qNED2BODY; qNED2BODY.setRPY(0.0, 0.0, 0.0);
    tf::Vector3 tNED2BODY; tNED2BODY.setValue(mav_state(0), mav_state(1), mav_state(2));

    // transform world frame to vehicle frame
    transform.setRotation(qNED2BODY);
    transform.setOrigin(tNED2BODY);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_ned", "vehicle"));

    // transform vehicle frame to vehicle1 frame
    tNED2BODY.setValue(0., 0., 0.);
    qNED2BODY.setRPY(0., 0., mav_state(5));
    transform.setIdentity();
    transform.setRotation(qNED2BODY);
    transform.setOrigin(tNED2BODY);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "vehicle", "vehicle1"));

    // transform vehicle1 frame to body
    tNED2BODY.setValue(0., 0., 0.);
    qNED2BODY.setRPY(mav_state(3), mav_state(4), 0.);
    transform.setIdentity();
    transform.setRotation(qNED2BODY);
    transform.setOrigin(tNED2BODY);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "vehicle1", "base_link"));

    // Publish the velocities
    geometry_msgs::Twist msg; 
    msg.linear.x = mav_state(6); msg.linear.y = mav_state(7); msg.linear.z = mav_state(8);
    msg.angular.x = mav_state(9); msg.angular.y = mav_state(10); msg.angular.z = mav_state(11);
    twist_pub_.publish(msg);
  }

  void MavDynamics::RK4(double dt)
  {
    Vector12d k_1 = dynamics(mav_state);
    Vector12d k_2 = dynamics(mav_state + (dt/2.0)*k_1);
    Vector12d k_3 = dynamics(mav_state + (dt/2.0)*k_2);
    Vector12d k_4 = dynamics(mav_state + dt*k_3);
    mav_state += (dt/6.0)*(k_1 + 2.0*k_2 + 2.0*k_3 + k_4);

    if (mav_state(2) > 0)
    {
      mav_state(2) = 0.0;
      mav_state(3) = 0.0;
      Eigen::Vector3d vel;
      vel << mav_state(6), mav_state(7), mav_state(8);
      Eigen::Quaternion<double> R_vb (Eigen::AngleAxisd(mav_state(3), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(mav_state(4), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(mav_state(5), Eigen::Vector3d::UnitZ())); 
      // Put it in vehicle frame
      vel = R_vb*vel;
      if (vel(2) > 0);
      {
        vel(2) = -0.3*vel(2);
      }
      vel = R_vb.inverse()*vel;
      mav_state(6) = vel(0); mav_state(7) = vel(1); mav_state(8) = vel(2);
    }
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

    //ROS_ERROR_STREAM(vel);

    double cphi = std::cos(att(0));
    double sphi = std::sin(att(0));
    double ctheta = std::cos(att(1));
    double stheta = std::sin(att(1));
    double ttheta = std::tan(att(1));
    double cpsi = std::cos(att(2));
    double spsi = std::sin(att(2));


    // Transformation to convert body into vehicle (i.e. NED)
    // note: vehicle to body is given, then we take the inverse, but Eigen uses active rotations
    // instead of passive rotations so we take the inverse again
    Eigen::Quaternion<double> R_vb (Eigen::AngleAxisd(att(0), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(att(1), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(att(2), Eigen::Vector3d::UnitZ())); 
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
