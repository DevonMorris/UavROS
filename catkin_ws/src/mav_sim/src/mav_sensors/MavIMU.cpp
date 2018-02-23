#include "mav_sensors/MavIMU.h"

namespace mav_imu
{
  MavIMU::MavIMU() :
    nh_(ros::NodeHandle()),
    params_(nh_),
    rd(),
    gen(rd()),
    d(0,1)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // Initialize forces/angular vels to 0
    force = Eigen::Vector3f::Zero();
    omega = Eigen::Vector3f::Zero();

    // publishers and subscribes
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/mav/imu", 5);
    twist_sub_ = nh_.subscribe("/mav/twist", 5, &MavIMU::twist_cb_, this);
    wrench_sub_ = nh_.subscribe("/mav/wrench", 5, &MavIMU::wrench_cb_, this);

    // setup std dev for acc
    sig_ax = .0025*params_.g;
    sig_ay = .0025*params_.g;
    sig_az = .0025*params_.g;

    // setup std dev for gyro
    sig_p = .002269;
    sig_q = .002269;
    sig_r = .002269;
  }

  void MavIMU::wrench_cb_(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
  }

  void MavIMU::twist_cb_(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    omega << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
  }

  void MavIMU::tick()
  {
    // find timestep
    double dt = (ros::Time::now() - now).toSec();
    now = ros::Time::now();

    // Get transform from TF tree
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

    Eigen::Vector3f F_g;
    F_g << 0.0, 0.0, params_.m*params_.g;
    tf::quaternionTFToEigen(tf_bv.getRotation(), R_bv);
    F_g = R_bv.cast <float>()*F_g;
    
    Eigen::Vector3f F_total = force - F_g;
    Eigen::Vector3f acc = F_total/params_.m;

    // add noise
    acc(0) += sig_ax*d(gen);
    acc(1) += sig_ay*d(gen);
    acc(2) += sig_az*d(gen);

    omega(0) += sig_p*d(gen);
    omega(1) += sig_q*d(gen);
    omega(2) += sig_r*d(gen);

    // pack up message and publish
    sensor_msgs::Imu imu_msg; 
    imu_msg.linear_acceleration.x = acc(0);
    imu_msg.linear_acceleration.y = acc(1);
    imu_msg.linear_acceleration.z = acc(2);

    imu_msg.angular_velocity.x = omega(0);
    imu_msg.angular_velocity.y = omega(1);
    imu_msg.angular_velocity.z = omega(2);

    imu_msg.header.stamp = ros::Time::now();

    imu_pub_.publish(imu_msg);
  }


}
