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
    mav_state = Eigen::MatrixXf::Zero(12,1);
    // Initialize forces/torques to 0
    force = Eigen::Vector3f::Zero();
    torque = Eigen::Vector3f::Zero();

    // Create inertia matrix
    J << params_.Jx, 0, -params_.Jxz,
         0,     params_.Jy,        0,
         -params_.Jxz, 0, params_.Jz;

    J_inv = J.inverse();

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
    double dt = (ros::Time::now() - now).toSec();
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
    Vector12f k_1 = dynamics(mav_state);
    Vector12f k_2 = dynamics(mav_state + (dt/2.0)*k_1);
    Vector12f k_3 = dynamics(mav_state + (dt/2.0)*k_2);
    Vector12f k_4 = dynamics(mav_state + dt*k_3);
    mav_state += (dt/6.0)*(k_1 + 2.0*k_2 + 2.0*k_3 + k_4);

    if (mav_state(2) > 0)
    {
      Eigen::Vector3f vel;
      vel << mav_state(6), mav_state(7), mav_state(8);
      Eigen::Quaternion<float> R_vb (Eigen::AngleAxisf(mav_state(3), Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(mav_state(4), Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(mav_state(5), Eigen::Vector3f::UnitZ())); 
      // Put it in vehicle frame
      vel = R_vb*vel;
      if (vel(2) > 0);
      {
        vel(2) = -0.6*vel(2);
      }
      vel = R_vb.inverse()*vel;
      mav_state(6) = vel(0); mav_state(7) = vel(1); mav_state(8) = vel(2);

      mav_state(2) = 0.0;
    }
  }

  Vector12f MavDynamics::dynamics(Vector12f state)
  {
    Vector12f state_dot = Eigen::MatrixXf::Zero(12,1);

    //state derivatives
    Eigen::Vector3f dpos;
    Eigen::Vector3f datt;
    Eigen::Vector3f dvel;
    Eigen::Vector3f domega;

    //unpack state
    Eigen::Vector3f pos;
    Eigen::Vector3f att;
    Eigen::Vector3f vel;
    Eigen::Vector3f omega;

    pos << state(0), state(1), state(2);
    att << state(3), state(4), state(5);
    vel << state(6), state(7), state(8);
    omega << state(9), state(10), state(11);

    //ROS_ERROR_STREAM(vel);

    float cphi = std::cos(att(0));
    float sphi = std::sin(att(0));
    float ctheta = std::cos(att(1));
    float stheta = std::sin(att(1));
    float ttheta = std::tan(att(1));
    float cpsi = std::cos(att(2));
    float spsi = std::sin(att(2));


    // Transformation to convert body into vehicle (i.e. NED)
    // note: vehicle to body is given, then we take the inverse, but Eigen uses active rotations
    // instead of passive rotations so we take the inverse again
    Eigen::Quaternion<float> R_vb (Eigen::AngleAxisf(att(0), Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(att(1), Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(att(2), Eigen::Vector3f::UnitZ())); 

    // Make matrix for datt
    Eigen::Matrix3f AttD;

    AttD << 1.0, sphi*ttheta, cphi*ttheta,
            0.0, cphi, -sphi,
            0.0, sphi/ctheta, cphi/ctheta;


    // Calculate derivatives
    dpos = R_vb*vel;
    dvel = -omega.cross(vel) + force/params_.m;
    datt = AttD*omega;
    domega = J_inv*(-omega.cross(J*omega)+torque);

    state_dot << dpos, datt, dvel, domega;

    return state_dot;
  }

}
