#include <turtlebot_slam/mapping.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_mapping_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  turtlebot::slam::Mapping mapper(nh, priv_nh);
  mapper.spin();

  return 0;
}