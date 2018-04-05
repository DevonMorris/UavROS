#include <ros/ros.h>
#include <mav_msgs/Waypoint.h>

#define num_waypoints 4

int main(int argc, char **argv)
{
  ros::init(argc, argv, "super_simple_path_planner");

  ros::NodeHandle nh_;
  ros::Publisher waypointPublisher = nh_.advertise<mav_msgs::Waypoint>("/mav/waypoint", 10);

  ros::Duration(1).sleep();

  float Va = 35;
  float wps[5*num_waypoints] =
  {
    0, 0, -200, 0, Va,
    1200, 0, -200, 45*M_PI/180, Va,
    0, 1200, -200, 45*M_PI/180, Va,
    1200, 1200, -200, -135*M_PI/180, Va,
  };

  for (int i(0); i < num_waypoints; i++)
  {
    ros::Duration(0.5).sleep();

    mav_msgs::Waypoint new_waypoint;

    new_waypoint.ned.x = wps[i*5 + 0];
    new_waypoint.ned.y = wps[i*5 + 1];
    new_waypoint.ned.z = wps[i*5 + 2];
    new_waypoint.chi_d = wps[i*5 + 3];
    new_waypoint.Va_d = wps[i*5 + 4];

    waypointPublisher.publish(new_waypoint);
  }

  ros::Duration(1.5).sleep();

  return 0;
}
