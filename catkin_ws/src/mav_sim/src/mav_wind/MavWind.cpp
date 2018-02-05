#include "mav_wind/MavWind.h"

namespace mav_wind
{
  MavWind::MavWind():
    nh_(ros::NodeHandle()),
    rd(),
    gen(rd()),
    d(0,1)
  {
    // publishers and subscribers
    wind_pub_ = nh_.advertise<geometry_msgs::Vector3>("/mav/wind", 5);
    steady_wind_sub_ = nh_.subscribe("/steady_wind", 5, &MavWind::steady_wind_cb_, this);

    // Initialize wind
    wind = Eigen::Vector3f::Zero();
    wind_b = Eigen::Vector3f::Zero();
    gust = Eigen::Vector3f::Zero();
    rand = Eigen::Vector3f::Zero();
    rand1 = Eigen::Vector3f::Zero();
    rand2 = Eigen::Vector3f::Zero();
    gust1 = Eigen::Vector3f::Zero();
    gust2 = Eigen::Vector3f::Zero();

  }

  void MavWind::calcWind()
  {
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

    tf::quaternionTFToEigen(tf_bv.getRotation(), R_bv);
    wind_b = R_bv.cast <float>()*wind;

    rand << d(gen), d(gen), d(gen);
    // These transfer functions were created using the dryden gust model
    // and Matlab's c2d command
    gust(0) = 0.9983*gust1(0) + 0.006266*rand(0) + 0.006266*rand1(0);
    //gust(1) = 1.997*gust1(1) - 0.9965*gust2(1) + 0.007671*rand(1) + 7.746e-6*rand1(1) - 0.007663*rand2(1);
    // For some reason the params in the book don't work for y
    gust(1) = 1.986*gust1(1) - 0.9861*gust2(1) + 0.01009*rand(1) + 4.071e-5*rand1(1) - 0.01005*rand2(1);
    gust(2) = 1.986*gust1(2) - 0.9861*gust2(2) + 0.01009*rand(2) + 4.071e-5*rand1(2) - 0.01005*rand2(2);

    // delay inputs and outputs
    gust2 = gust1;
    gust1 = gust;
    rand2 = rand1;
    rand1 = rand;

    wind_b = wind_b + gust;
  }
  void MavWind::steady_wind_cb_(const geometry_msgs::Vector3ConstPtr& msg)
  {
    wind << msg->x, msg->y, msg->z;
  }

  void MavWind::tick()
  {
    calcWind();
    geometry_msgs::Vector3 msg;

    // pack up message and publish
    msg.x = wind_b(0); msg.y = wind_b(1); msg.z = wind_b(2);
    wind_pub_.publish(msg);
  }

}
