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
    manager = managers::FILLET;
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
    path_msg.Vg = waypoints[idx_a].Va_d;
    path_msg.line = true;

    if ((p - w_1).dot(n) > 0)
    {
      idx_a++;
      idx_a = idx_a % n_waypoints;
    }

  }

  void MavPathManager::manage_fillet(mav_msgs::Path& path_msg)
  {
    int idx_b;
    int idx_c;
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

    float R_min = 100;

    path_msg.Vg = waypoints[idx_a].Va_d;
    path_msg.r.x = w_0(0); path_msg.r.y = w_0(1); path_msg.r.z = w_0(2);
    Eigen::Vector3f q_0 = (w_1 - w_0).normalized();
    Eigen::Vector3f q_1 = (w_2 - w_1).normalized();
    float beta = std::acos(-q_0.dot(q_1));

    Eigen::Vector3f z;
    switch (fil_state_)
    {
    case fillet_state::STRAIGHT:
      path_msg.line = true;
      path_msg.q.x = q_0(0); path_msg.q.y = q_0(1); path_msg.q.z = q_0(2);
      path_msg.c.x = 1; path_msg.c.y = 1; path_msg.c.z = 1;
      path_msg.rho = 1;
      path_msg.lamb = 1;
      z = w_1 - q_0*(R_min/std::tan(beta/2.0));
      if ((p - z).dot(q_0) > 0)
        fil_state_ = fillet_state::ORBIT;
      break;
    case fillet_state::ORBIT:
      path_msg.line = false;
      path_msg.q.x = q_1(0); path_msg.q.y = q_1(1); path_msg.q.z = q_1(2);
      Eigen::Vector3f c = w_0 - (q_0 - q_1).normalized()*(R_min/std::sin(beta/2.0));
      path_msg.c.x = c(0); path_msg.c.y = c(1); path_msg.c.z = c(2);
      path_msg.rho = R_min;
      path_msg.lamb = ((q_0(0)*q_1(1) - q_0(1)*q_1(0)) > 0 ? 1 : -1);
      z = w_1 + q_1*(R_min/std::tan(beta/2.0));
      if ((p - z).dot(q_1) > 0)
      {
        idx_a++;
        idx_a = idx_a % n_waypoints;
        fil_state_ = fillet_state::STRAIGHT;
      }
      break;
    }
    return;
  }

  void MavPathManager::manage_dubins(mav_msgs::Path& path_msg)
  {
    path_msg.Vg = waypoints[idx_a].Va_d;
    path_msg.r.x = 0; path_msg.r.y = 0; path_msg.r.z = 0;
    path_msg.q.x = 0; path_msg.q.y = 0; path_msg.q.z = 0;
    path_msg.c.x = 0; path_msg.c.y = 0; path_msg.c.z = 0;

    switch (dub_state_)
    {
    case dubin_state::FIRST:
      dubinsParameters(waypoints[0], waypoints[1], 100);
      path_msg.line = false;
      path_msg.c.x = dubinspath_.cs(0); path_msg.c.y = dubinspath_.cs(1); path_msg.c.z = dubinspath_.cs(2);
      path_msg.rho = dubinspath_.R; path_msg.lamb = dubinspath_.lams;
      if ((p - dubinspath_.w1).dot(-dubinspath_.q1) >= 0) // start in H1
      {
        dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
      }
      else
      {
        dub_state_ = dubin_state::BEFORE_H1;
      }
      break;
    case dubin_state::BEFORE_H1:
      path_msg.line = false;
      path_msg.c.x = dubinspath_.cs(0); path_msg.c.y = dubinspath_.cs(1); path_msg.c.z = dubinspath_.cs(2);
      path_msg.rho = dubinspath_.R; path_msg.lamb = dubinspath_.lams;
      if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // entering H1
      {
        dub_state_ = dubin_state::STRAIGHT;
      }
      break;
    case dubin_state::BEFORE_H1_WRONG_SIDE:
      path_msg.line = false;
      path_msg.c.x = dubinspath_.cs(0); path_msg.c.y = dubinspath_.cs(1); path_msg.c.z = dubinspath_.cs(2);
      path_msg.rho = dubinspath_.R; path_msg.lamb = dubinspath_.lams;
      if ((p - dubinspath_.w1).dot(dubinspath_.q1) < 0) // exit H1
      {
        dub_state_ = dubin_state::BEFORE_H1;
      }
      break;
    case dubin_state::STRAIGHT:
      path_msg.line = true;
      path_msg.r.x = dubinspath_.w1(0); path_msg.r.y = dubinspath_.w1(1); path_msg.r.z = dubinspath_.w1(2);
      path_msg.q.x = dubinspath_.q1(0); path_msg.q.y = dubinspath_.q1(1); path_msg.q.z = dubinspath_.q1(2);
      path_msg.rho = 1; path_msg.lamb = 1;
      if ((p - dubinspath_.w2).dot(dubinspath_.q1) >= 0) // entering H2
      {
        if ((p - dubinspath_.w3).dot(dubinspath_.q3) >= 0) // start in H3
        {
          dub_state_ = dubin_state::BEFORE_H3_WRONG_SIDE;
        }
        else
        {
          dub_state_ = dubin_state::BEFORE_H3;
        }
      }
      break;
    case dubin_state::BEFORE_H3:
      path_msg.line = false;
      path_msg.c.x = dubinspath_.ce(0); path_msg.c.y = dubinspath_.ce(1); path_msg.c.z = dubinspath_.ce(2);
      path_msg.rho = dubinspath_.R; path_msg.lamb = dubinspath_.lame;
      if ((p - dubinspath_.w3).dot(dubinspath_.q3) >= 0) // entering H3
      {
        // increase the waypoint pointer
        int idx_b;
        if (idx_a == n_waypoints - 1)
        {
          idx_a = 0;
          idx_b = 1;
        }
        else if (idx_a == n_waypoints - 2)
        {
          idx_a++;
          idx_b = 0;
        }
        else
        {
          idx_a++;
          idx_b = idx_a + 1;
        }

        // plan new Dubin's path to next waypoint configuration
        dubinsParameters(waypoints[idx_a], waypoints[idx_b], 100);

        //start new path
        if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // start in H1
        {
          dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
        }
        else
        {
          dub_state_ = dubin_state::BEFORE_H1;
        }
      }
      break;
    case dubin_state::BEFORE_H3_WRONG_SIDE:
      path_msg.line = false;
      path_msg.c.x = dubinspath_.ce(0); path_msg.c.y = dubinspath_.ce(1); path_msg.c.z = dubinspath_.ce(2);
      path_msg.rho = dubinspath_.R; path_msg.lamb = dubinspath_.lame;
      if ((p - dubinspath_.w3).dot(dubinspath_.q3) < 0) // exit H3
      {
        dub_state_ = dubin_state::BEFORE_H1;
      }
      break;
    }
    return;
  }

  void MavPathManager::dubinsParameters(const waypoint start_node, const waypoint end_node, float R)
  {
    float ell = std::sqrt((start_node.ned(0) - end_node.ned(0))*(start_node.ned(0) - end_node.ned(0)) +
                      (start_node.ned(1) - end_node.ned(1))*(start_node.ned(1) - end_node.ned(1)));
    if (ell < 2.0*R)
    {
      ROS_ERROR("The distance between nodes must be larger than 2R.");
    }
    else
    {
      dubinspath_.ps(0) = start_node.ned(0);
      dubinspath_.ps(1) = start_node.ned(1);
      dubinspath_.ps(2) = start_node.ned(2);
      dubinspath_.chis = start_node.chi_d;
      dubinspath_.pe(0) = end_node.ned(0);
      dubinspath_.pe(1) = end_node.ned(1);
      dubinspath_.pe(2) = end_node.ned(2);
      dubinspath_.chie = end_node.chi_d;

      Eigen::Vector3f crs = dubinspath_.ps;
      crs(0) += R*(std::cos(M_PI/2)*std::cos(dubinspath_.chis) - std::sin(M_PI/2)*std::sin(dubinspath_.chis));
      crs(1) += R*(std::sin(M_PI/2)*std::cos(dubinspath_.chis) + std::cos(M_PI/2)*std::sin(dubinspath_.chis));
      Eigen::Vector3f cls = dubinspath_.ps;
      cls(0) += R*(std::cos(-M_PI/2)*std::cos(dubinspath_.chis) - std::sin(-M_PI/2)*std::sin(dubinspath_.chis));
      cls(1) += R*(std::sin(-M_PI/2)*std::cos(dubinspath_.chis) + std::cos(-M_PI/2)*std::sin(dubinspath_.chis));
      Eigen::Vector3f cre = dubinspath_.pe;
      cre(0) += R*(std::cos(M_PI/2)*std::cos(dubinspath_.chie) - std::sin(M_PI/2)*std::sin(dubinspath_.chie));
      cre(1) += R*(std::sin(M_PI/2)*std::cos(dubinspath_.chie) + std::cos(M_PI/2)*std::sin(dubinspath_.chie));
      Eigen::Vector3f cle = dubinspath_.pe;
      cle(0) += R*(std::cos(-M_PI/2)*std::cos(dubinspath_.chie) - std::sin(-M_PI/2)*std::sin(dubinspath_.chie));
      cle(1) += R*(std::sin(-M_PI/2)*std::cos(dubinspath_.chie) + std::cos(-M_PI/2)*std::sin(dubinspath_.chie));

      float theta, theta2;
      // compute L1
      theta = std::atan2(cre(1) - crs(1), cre(0) - crs(0));
      float L1 = (crs - cre).norm() + R*mo(2.0*M_PI + mo(theta - M_PI/2) - mo(dubinspath_.chis - M_PI/2))
                 + R*mo(2.0*M_PI + mo(dubinspath_.chie - M_PI/2) - mo(theta - M_PI/2));

      // compute L2
      ell = (cle - crs).norm();
      theta = std::atan2(cle(1) - crs(1), cle(0) - crs(0));
      float L2;
      if (2.0*R > ell)
        L2 = 9999.0f;
      else
      {
        theta2 = theta - M_PI/2 + std::asin(2.0*R/ell);
        L2 = sqrtf(ell*ell - 4.0*R*R) + R*mo(2.0*M_PI + mo(theta2) - mo(dubinspath_.chis - M_PI/2))
             + R*mo(2.0*M_PI + mo(theta2 + M_PI) - mo(dubinspath_.chie + M_PI/2));
      }

      // compute L3
      ell = (cre - cls).norm();
      theta = std::atan2(cre(1) - cls(1), cre(0) - cls(0));
      float L3;
      if (2.0*R > ell)
        L3 = 9999.0f;
      else
      {
        theta2 = std::acos(2.0*R/ell);
        L3 = std::sqrt(ell*ell - 4*R*R) + R*mo(2.0*M_PI + mo(dubinspath_.chis + M_PI/2) - mo(theta + theta2))
             + R*mo(2.0*M_PI + mo(dubinspath_.chie - M_PI/2) - mo(theta + theta2 - M_PI));
      }

      // compute L4
      theta = std::atan2(cle(1) - cls(1), cle(0) - cls(0));
      float L4 = (cls - cle).norm() + R*mo(2.0*M_PI + mo(dubinspath_.chis + M_PI/2) - mo(theta + M_PI/2))
                 + R*mo(2.0*M_PI + mo(theta + M_PI/2) - mo(dubinspath_.chie + M_PI/2));

      // L is the minimum distance
      int idx = 1;
      dubinspath_.L = L1;
      if (L2 < dubinspath_.L)
      {
        dubinspath_.L = L2;
        idx = 2;
      }
      if (L3 < dubinspath_.L)
      {
        dubinspath_.L = L3;
        idx = 3;
      }
      if (L4 < dubinspath_.L)
      {
        dubinspath_.L = L4;
        idx = 4;
      }

      Eigen::Vector3f e1;
      //        e1.zero();
      e1(0) = 1;
      e1(1) = 0;
      e1(2) = 0;
      switch (idx)
      {
      case 1:
        dubinspath_.cs = crs;
        dubinspath_.lams = 1;
        dubinspath_.ce = cre;
        dubinspath_.lame = 1;
        dubinspath_.q1 = (cre - crs).normalized();
        dubinspath_.w1 = dubinspath_.cs + (rotz(-M_PI/2)*dubinspath_.q1)*R;
        dubinspath_.w2 = dubinspath_.ce + (rotz(-M_PI/2)*dubinspath_.q1)*R;
        break;
      case 2:
        dubinspath_.cs = crs;
        dubinspath_.lams = 1;
        dubinspath_.ce = cle;
        dubinspath_.lame = -1;
        ell = (cle - crs).norm();
        theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
        theta2 = theta - M_PI/2 + std::asin(2.0*R/ell);
        dubinspath_.q1 = rotz(theta2 + M_PI/2)*e1;
        dubinspath_.w1 = dubinspath_.cs + (rotz(theta2)*e1)*R;
        dubinspath_.w2 = dubinspath_.ce + (rotz(theta2 + M_PI)*e1)*R;
        break;
      case 3:
        dubinspath_.cs = cls;
        dubinspath_.lams = -1;
        dubinspath_.ce = cre;
        dubinspath_.lame = 1;
        ell = (cre - cls).norm();
        theta = std::atan2(cre(1) - cls(1), cre(0) - cls(0));
        theta2 = std::acos(2.0*R/ ell);
        dubinspath_.q1 = rotz(theta + theta2 - M_PI/2)*e1;
        dubinspath_.w1 = dubinspath_.cs + (rotz(theta + theta2)*e1)*R;
        dubinspath_.w2 = dubinspath_.ce + (rotz(theta + theta2 - M_PI)*e1)*R;
        break;
      case 4:
        dubinspath_.cs = cls;
        dubinspath_.lams = -1;
        dubinspath_.ce = cle;
        dubinspath_.lame = -1;
        dubinspath_.q1 = (cle - cls).normalized();
        dubinspath_.w1 = dubinspath_.cs + (rotz(M_PI/2)*dubinspath_.q1)*R;
        dubinspath_.w2 = dubinspath_.ce + (rotz(M_PI/2)*dubinspath_.q1)*R;
        break;
      }
      dubinspath_.w3 = dubinspath_.pe;
      dubinspath_.q3 = rotz(dubinspath_.chie)*e1;
      dubinspath_.R = R;
    }
  }

  float MavPathManager::mo(float in)
  {
    float val;
    if (in > 0)
      val = std::fmod(in,2.0*M_PI);
    else
    {
      float n = std::floor(in/2.0/M_PI);
      val = in - n*2.0*M_PI;
    }
    return val;
  }

  Eigen::Matrix3f MavPathManager::rotz(float theta)
  {
    Eigen::Matrix3f R;
    R << std::cos(theta), -std::sin(theta), 0,
    std::sin(theta),  std::cos(theta), 0,
    0,            0, 1;

    return R;
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
      path_msg.rho = 300;
      path_msg.lamb = 1;
    }

    else
    {
      switch(manager)
      {
        case managers::LINE:
          manage_line(path_msg);
          break;
        case managers::FILLET:
          manage_fillet(path_msg);
          break;
        case managers::DUBINS:
          manage_dubins(path_msg);
          break;
      }
    }

    path_pub_.publish(path_msg);

  }

}
