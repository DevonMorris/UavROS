#include "mav_path_manager/MavPathManager.h"

namespace mav_path_manager
{
  MavPathManager::MavPathManager() :
    nh_(ros::NodeHandle()),
    params_(nh_)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // publishers and subscribes
    path_pub_ = nh_.advertise<mav_msgs::Path>("/mav/path", 5);

    waypoint_sub_ = nh_.subscribe("/mav/waypoint", 5, &MavPathManager::waypoint_cb_, this);
    ned_est_sub_ = nh_.subscribe("/mav/ned_est", 5, &MavPathManager::ned_est_cb_, this);

    fil_state_ = fillet_state::STRAIGHT;
    dub_state_ = dubin_state::FIRST;

    waypoints.clear();

    p = Eigen::Vector3f::Zero();
    p(2) = -200;
    n_waypoints = 0;
    idx_a = 0;

    // change this to change path manager type
    manager = managers::LINE;
  }

  void MavPathManager::ned_est_cb_(const geometry_msgs::Vector3StampedConstPtr& msg)
  {
    p << msg->vector.x, msg->vector.y, msg->vector.z;
  }

  void MavPathManager::waypoint_cb_(const mav_msgs::WaypointConstPtr& msg)
  {
    waypoint nextwp; 
    nextwp.ned << msg->ned.x, msg->ned.y, msg->ned.z;
    nextwp.chi_d = msg->chi_d;
    nextwp.Va_d = msg->Va_d; 

    waypoints.push_back(nextwp);
    n_waypoints++;
  }

  void MavPathManager::manage_line(mav_msgs::Path& path_msg)
  {
    int idx_b;
    int idx_c;

    // this allows for wrapping the waypoints
    if (idx_a == n_waypoints - 1)
    {
      idx_b = 0;
      idx_c = 1;
    }
    else if (idx_a == n_waypoints - 2)
    {
      idx_b = n_waypoints - 1;
      idx_c = 0;
    }
    else
    {
      idx_b = idx_a + 1;
      idx_c = idx_b + 1;
    }

    Eigen::Vector3f w_0(waypoints[idx_a].ned);
    Eigen::Vector3f w_1(waypoints[idx_b].ned);
    Eigen::Vector3f w_2(waypoints[idx_c].ned);

    Eigen::Vector3f q_0((w_1 - w_0).normalized());
    Eigen::Vector3f q_1((w_2 - w_1).normalized());
    Eigen::Vector3f n((q_0 + q_1).normalized());

    path_msg.r.x = w_0(0); path_msg.r.y = w_0(1); path_msg.r.z = w_0(2);
    path_msg.q.x = q_0(0); path_msg.q.y = q_0(1); path_msg.q.z = q_0(2);
    path_msg.line = true;

    if ((p - w_1).dot(n) > 0)
    {
      idx_a++;
      idx_a = idx_a % n_waypoints;
    }

  }

  void MavPathManager::manage_fillet(mav_msgs::Path& path_msg)
  {
    return;
  }

  void MavPathManager::manage_dubins(mav_msgs::Path& path_msg)
  {
    return;
  }

  void MavPathManager::tick()
  {
    mav_msgs::Path path_msg;
    if (n_waypoints < 3)
    {
      ROS_WARN_THROTTLE(5, "Too few waypoints received! Loitering about origin at 200m");
      path_msg.line = false;
      path_msg.Vg = 35;
      path_msg.c.x = 0.0; path_msg.c.y = 0.0; path_msg.c.z = -200;
      path_msg.rho = 400;
      path_msg.lamb = 1;
    }

    else
    {
      switch(manager)
      {
        case managers::LINE:
          manage_line(path_msg);
        case managers::FILLET:
          manage_fillet(path_msg);
        case managers::DUBINS:
          manage_dubins(path_msg);
      }
    }

    path_pub_.publish(path_msg);

  }

}
