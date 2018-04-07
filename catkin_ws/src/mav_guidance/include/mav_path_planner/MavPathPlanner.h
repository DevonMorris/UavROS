#pragma once

#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mav_msgs/Path.h>
#include <mav_msgs/Waypoint.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <limits>
#include <algorithm>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <mav_params/MavParams.h>

namespace mav_path_planner
{

struct RRTNode
{
  Eigen::Vector2d ne;
  std::shared_ptr<RRTNode> parent;
};

using RRTNodePtr = std::shared_ptr<RRTNode>;

struct Waypoint
{
  Eigen::Vector3f ned;
  float chi_d;
  float Va_d;
};

class MavPathPlanner
{
private:
  // Ros node handles, publishers and subscribers
  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub_;
  ros::Subscriber map_sub_;

  void map_cb_(const grid_map_msgs::GridMapConstPtr& msg);

  RRTNodePtr start;
  RRTNodePtr end;

  std::vector<RRTNodePtr> rrtree;
  std::vector<RRTNodePtr> waypoints;

  grid_map::GridMap map;

  ros::Time now;

  // random number generator
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;

  RRTNodePtr generateRandomConfig();
  RRTNodePtr findClosestConfig(RRTNodePtr N);
  RRTNodePtr planPath(RRTNodePtr N, RRTNodePtr M);
  bool checkFeasible(RRTNodePtr N, RRTNodePtr M);
  bool checkEnd(RRTNodePtr N);
  void smoothPath();
  void runRRT();

  float end_dist;

  bool map_init;
  bool initd;

  // params
  mav_params::MavParams params_;
  
public:
  bool planning;
  MavPathPlanner();
  void tick();
}; //end class MavPathPlanner

} //end namespace
