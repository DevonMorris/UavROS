#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

using namespace grid_map;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "mav_map");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("/mav/map", 1, true);

  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("world_ned");
  map.setGeometry(Length(2000, 2000), 10); // 
  map.setPosition(Position(1000, 1000));
  //ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
  //  map.getLength().x(), map.getLength().y(),
  //  map.getSize()(0), map.getSize()(1));

  // Work with grid map in a loop.
  ros::Rate rate(.25);
  while (nh.ok()) {

    // Add data to grid map.
    float n_blocks = 5; // number of city blocks
    float street_width = 200; // width of street in meters
    float grid_size = 1500 / n_blocks;
    ros::Time time = ros::Time::now();
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = 0;
      if ( std::fmod(position.x(), grid_size) > street_width)
      {
        if ( std::fmod(position.y(), grid_size) > street_width)
        {
          map.at("elevation", *it) = -300;
        }
      } 
    }

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}
