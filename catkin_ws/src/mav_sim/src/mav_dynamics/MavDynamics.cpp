#include <mav_dynamics/MavDynamics.h>

namespace mav_dynamics
{
  MavDynamics::MavDynamics() :
    nh_(ros::NodeHandle())
  {
      ros::NodeHandle nh_private("~");
      now = ros::Time::now();

      // Initialize state to 0
      mav_state = Eigen::MatrixXd::Zero(12);
      // Initialize forces/torques to 0
      force = Eigen::Vector3d::Zero();
      torque = Eigen::Vector3d::Zero();

      // publishers and subscribes
      input_sub_ = nh_.subscribe("/mav/input", 5, &MavDynamics::input_cb_, this);
  }

  void MavDynamics::input_cb_(const geometry_msgs::WrenchConstPtr& msg)
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
    tf::Quaternion qNED2BODY; qNED2BODY.setRPY(mav_state(3), mav_state(4), mav_state(5));
    tf::Vector3 tNED2BODY; tNED2BODY.setValue(mav_state(0), mav_state(1), mav_state(2));

    transform.setRotation(qNED2BODY);
    transform.setOrigin(tNED2BODY);
    tf_br_.sendTransform(tf::StampedTransform(transform, now, "world_ned", "base_link"));
  }

  void MavDynamics::RK4(double dt)
  {
    MavState k_1 = dynamics(mav_state);
    MavState k_2 = dynamics(mav_state + dt/2*k_1);
    MavState k_3 = dynamics(mav_state + dt/2*k_2);
    MavState k_4 = dynamics(mav_state + dt*k_3);
    mav_state += dt/6*(k_1 + 2*k_2 + 2*k_3 + k_4);
  }

  MavState dynamics(MavState state)
  {
    MavState state_dot = Eigen::MatrixXd::Zero(12);

    //unpack state
    Eigen::Vector3d pos;
    Eigen::Vector3d att;
    Eigen::Vector3d vel;
    Eigen::Vector3d omega;

    pos << state(0), state(1), state(2);
    att << state(3), state(4), state(5);
    att << state(6), state(7), state(8);
    att << state(9), state(10), state(11);

    double c_psi = cos(state(5) * 0.5);
    double s_psi = sin(state(5) * 0.5);
    double c_phi = cos(state(3) * 0.5);
    double s_phi = sin(state(3) * 0.5);
    double c_th = cos(state(4) * 0.5);
    double s_th = sin(state(4) * 0.5);

    return state_dot;
  }

}
