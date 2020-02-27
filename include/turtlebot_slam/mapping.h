#ifndef TURTLEBOT_SLAM_MAPPING_H
#define TURTLEBOT_SLAM_MAPPING_H

#include "turtlebot_slam/MappingConfig.h"

#include <dynamic_reconfigure/server.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <string>

namespace turtlebot
{
namespace slam
{
class Mapping
{
public:
  Mapping(ros::NodeHandle &nh,
          ros::NodeHandle &priv_nh);
  ~Mapping();

  void spinOnce();
  void spin();

  enum Boundary
  {
    RIGHT = 1,
    LEFT = 2,
    TOP = 4,
    BOTTOM = 8
  };

protected:
  // Nodehandles
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  // Gazebo robot name (to extract pose information)
  std::string robot_name_;

  // Robot frames
  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  // Publishers
  ros::Publisher map_pub_;
  ros::Publisher viz_pub_;

  // Subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber ips_sub_;
  ros::Subscriber scan_sub_;

  // Subscriber Callbacks
  void poseCallback(const gazebo_msgs::ModelStatesConstPtr& states);
  void ipsCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& pose);
  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);

  // Publisher functions
  void publishMap(const ros::TimerEvent& e);

  // Utility Functions
  void updateMap(geometry_msgs::Point scan_point);
  void adjustMapBounds(uint8_t violations, double expand = 5.0);
  void updateLogOdds(int16_t x_cell,
                     int16_t y_cell,
                     int16_t dx_hit,
                     int16_t dy_hit);
  int8_t logOddsToOccupancy(double x);
  std::pair<bool, uint8_t> inMapBounds(double x, double y);
  std::pair<bool, uint8_t> inMapBounds(int16_t cell_x, int16_t cell_y);
  std::pair<double, double> mapToWorld(int16_t cell_x, int16_t cell_y);
  std::pair<int16_t, int16_t> worldToMap(double x, double y);
private:
  bool debug_;

  // Dynamic Reconfigure Server
  dynamic_reconfigure::Server<turtlebot_slam::MappingConfig> dr_server_;

  // Log odds update parameters
  double close_hit_occupancy_probability_;
  double far_hit_occupancy_probability_;
  int hit_threshold_;
  double unknown_occupancy_odds_buffer_;
  double saturation_log_odds_occupied_;
  double saturation_log_odds_free_;

  // Process rate parameters
  double loop_hz_;
  int skip_n_scans_;

  // Robot Pose
  geometry_msgs::Point robot_position_;
  double robot_yaw_;

  // Mapping Parameters/Variables
  ros::Timer map_publish_timer_;
  double map_publish_period_;
  double map_resolution_;

  // Mapping data structures
  Eigen::MatrixXd grid_map_;
  int16_t origin_cell_x_;
  int16_t origin_cell_y_;

  // TF variables
  geometry_msgs::TransformStamped map_to_odom_tf_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::TransformListener tf_listener_;

  // Visualization Functions
  void drawBresenham(std::vector<std::pair<int16_t, int16_t>> coords);

  // Dynamic Reconfigure Callback
  void drCallback(turtlebot_slam::MappingConfig &config, uint32_t level);
};
}
}
#endif  // TURTLEBOT_SLAM_MAPPING_H