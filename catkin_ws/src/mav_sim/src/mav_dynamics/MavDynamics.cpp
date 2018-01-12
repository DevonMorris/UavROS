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

  void RK4(Eigen::Vector3d& vec, std::function<Eigen::Vector3d()> func)
  {
    Eigen::Vector3d k1 = func
  }

}
