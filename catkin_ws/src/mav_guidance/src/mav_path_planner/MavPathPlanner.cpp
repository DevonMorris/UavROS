#include "mav_path_planner/MavPathPlanner.h"

namespace mav_path_planner
{
  MavPathPlanner::MavPathPlanner() :
    nh_(ros::NodeHandle()),
    params_(nh_),
    gen(rd()),
    dis(0.0, 1.0)
  {
    ros::NodeHandle nh_private("~");
    now = ros::Time::now();

    // publishers and subscribes
    waypoint_pub_ = nh_.advertise<mav_msgs::Waypoint>("/mav/waypoint", 50);
    map_sub_ = nh_.subscribe("/mav/map", 5, &MavPathPlanner::map_cb_, this);

    rrtree.clear();
    waypoints.clear();

    start = std::make_shared<RRTNode>();
    end = std::make_shared<RRTNode>();
    start->ne << 0., 0.;
    start->parent = nullptr;
    end->ne << 2000., 2000.;

    planning = true;
    map_init = false;
    end_dist = 50;

    rrtree.push_back(start);
    initd = true;
  }

  void MavPathPlanner::map_cb_(const grid_map_msgs::GridMapConstPtr& msg)
  {
    ROS_INFO_STREAM("Received map");
    grid_map::GridMapRosConverter::fromMessage(*msg ,map);
    map_init = true;
    ROS_INFO_STREAM(map.getSize());
  }

  void MavPathPlanner::tick()
  {
    if (map_init && initd)
    {
      runRRT();
    }
    return;
  }

  void MavPathPlanner::runRRT()
  {
    bool connected = false;
    RRTNodePtr p;
    RRTNodePtr v_s;
    RRTNodePtr v_p;
    int ctr = 0;
    ROS_INFO_STREAM("Planning RRT");
    while (!connected)
    {
      p = generateRandomConfig();
      v_s = findClosestConfig(p);
      v_p = planPath(v_s, p);
      if(checkFeasible(v_s, v_p))
      {
        ctr += 1;
        v_p->parent = v_s;
        rrtree.push_back(v_p);
        if(checkEnd(v_p))
        {
          end->parent = v_p;
          rrtree.push_back(end);
          connected = true;
        }
      }
    }

    ROS_INFO_STREAM("Smoothing RRT");
    
    smoothPath();
    // publish waypoints here
    ROS_INFO_STREAM("Publishing Waypoints");
    for (auto w : waypoints)
    {
      mav_msgs::Waypoint w_msg;
      w_msg.ned.x = w->ne(0);
      w_msg.ned.y = w->ne(1);
      w_msg.ned.z = -250;
      w_msg.chi_d = 0;
      w_msg.Va_d = 35;

      waypoint_pub_.publish(w_msg);
    }


    planning = false;
  }

  RRTNodePtr MavPathPlanner::generateRandomConfig()
  {
    RRTNodePtr p = std::make_shared<RRTNode>();
    p->ne << end->ne(0)*dis(gen), end->ne(1)*dis(gen);
    return p;
  }

  RRTNodePtr MavPathPlanner::findClosestConfig(RRTNodePtr N)
  {
    float d = 0;
    float d_min = std::numeric_limits<float>::infinity(); // big number
    RRTNodePtr closest_n;
    for (auto & n : rrtree)
    {
      d = (n->ne - N->ne).norm();
      if (d < d_min)
      {
        closest_n = n;
        d_min = d;
      }
    }
    return closest_n;
  }

  RRTNodePtr MavPathPlanner::planPath(RRTNodePtr N, RRTNodePtr M)
  {
    RRTNodePtr p = std::make_shared<RRTNode>();
    p->ne << N->ne + dis(gen)*(M->ne - N->ne);
    return p;
  }

  bool MavPathPlanner::checkFeasible(RRTNodePtr N, RRTNodePtr M)
  {
    grid_map::Position start_map, end_map;

    start_map << N->ne;
    end_map << M->ne;
    bool feasible = true;

    for (grid_map::LineIterator iterator(map, start_map, end_map);
        !iterator.isPastEnd(); ++iterator)
    {
      if (map.at("elevation", *iterator) < 0)
      {
        feasible = false;
      }
    }
    return feasible;
  }

  bool MavPathPlanner::checkEnd(RRTNodePtr N)
  {
    bool feasible = false;
    if (checkFeasible(N,end) && ((N->ne - end->ne).norm() < end_dist))
    {
      feasible = true;
    }
    return feasible;
  }

  void MavPathPlanner::smoothPath()
  {
    RRTNodePtr p = end;
    RRTNodePtr q = end->parent;
    waypoints.push_back(end);

    while (p->parent)
    {
      if (bool(q->parent) && checkFeasible(p, q->parent))
      {
        q = q->parent;
      }
      else
      {
        waypoints.push_back(q);
        p = q;
        q = q->parent;
      }
    }
    std::reverse(std::begin(waypoints), std::end(waypoints));
  }

}
