#pragma once

#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mav_msgs/Path.h>
#include <mav_msgs/Waypoint.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include <mav_params/MavParams.h>

namespace mav_path_manager
{

struct waypoint
{
  Eigen::Vector3f ned;
  float chi_d;
  float Va_d;
};

enum class fillet_state
{
  STRAIGHT,
  ORBIT
};

enum class dubin_state
{
  FIRST,
  BEFORE_H1,
  BEFORE_H1_WRONG_SIDE,
  STRAIGHT,
  BEFORE_H3,
  BEFORE_H3_WRONG_SIDE
};

enum class managers
{
  LINE,
  FILLET,
  DUBINS
};

class MavPathManager
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Publisher path_pub_;

  ros::Subscriber waypoint_sub_;
  ros::Subscriber ned_est_sub_;

  std::vector<waypoint> waypoints;
  int n_waypoints;
  int idx_a;

  ros::Time now;

  // callbacks for subs
  void waypoint_cb_(const mav_msgs::WaypointConstPtr& msg);
  void ned_est_cb_(const geometry_msgs::Vector3StampedConstPtr& msg);

  void computeLine(const mav_msgs::PathConstPtr& msg);
  void computeOrbit(const mav_msgs::PathConstPtr& msg);

  // Command variables
  Eigen::Vector3f p;

  void manage_line(mav_msgs::Path& path_msg);
  void manage_fillet(mav_msgs::Path& path_msg);
  void manage_dubins(mav_msgs::Path& path_msg);

  mav_msgs::Path c_path;

  dubin_state dub_state_;
  fillet_state fil_state_;

  managers manager;


  // params
  mav_params::MavParams params_;
  
public:
  MavPathManager();
  void tick();
}; //end class MavPathManager

} //end namespace
