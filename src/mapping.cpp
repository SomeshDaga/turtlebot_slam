#include "turtlebot_slam/mapping.h"
#include "turtlebot_slam/MappingConfig.h"

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>

#include <boost/bind.hpp>
#include <cmath>

namespace turtlebot
{
namespace slam
{
Mapping::Mapping(ros::NodeHandle& nh,
                 ros::NodeHandle& priv_nh)
  : nh_(nh),
    priv_nh_(priv_nh),
    tf_buffer_(),
    tf_listener_(tf_buffer_)
{
  // Set Dynamic Reconfigure Callback
  dr_server_.setCallback(boost::bind(&Mapping::drCallback, this, _1, _2));

  // Get Parameters from the Parameter Server
  priv_nh_.param<bool>("debug", debug_, false);
  priv_nh_.param<double>("loop_hz", loop_hz_, 10.0);

  // Mapping parameters
  priv_nh_.param<double>("resolution", map_resolution_, 0.05);
  priv_nh_.param<std::string>("base_frame", base_frame_, "base_link");
  priv_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
  priv_nh_.param<std::string>("map_frame", map_frame_, "map");
  priv_nh_.param<std::string>("robot_name", robot_name_, "mobile_base");
  priv_nh_.param<double>("map_publish_period", map_publish_period_, 5.0);

  // Align the map and odom frame
  map_to_odom_tf_.header.frame_id = map_frame_;
  map_to_odom_tf_.child_frame_id = odom_frame_;
  // Initialize the map to odom transform to the identity matrix (i.e. frames are aligned)
  map_to_odom_tf_.transform = tf2::getTransformIdentity<geometry_msgs::Transform>();

  // Let TFs be processed in a separate thread
  tf_buffer_.setUsingDedicatedThread(true);

  // Initialize publishers
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  viz_pub_ = nh_.advertise<visualization_msgs::Marker>("bresenham_visualization", 1, true);

  // Initialize subscribers
  pose_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 1, &Mapping::poseCallback, this);
  ips_sub_ = nh_.subscribe("indoor_pos", 1, &Mapping::ipsCallback, this);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &Mapping::scanCallback, this);

  // Initialize the grid map to a default size of 20m x 20m
  // The grid map will be expanded/resized if needed
  uint16_t map_size = ceil(5.0 / map_resolution_);
  grid_map_ = Eigen::MatrixXd::Zero(map_size, map_size);

  // Set the origin of the map to be in the (approximately) center of the grid map
  // Note that the origin point will be considered to be the center point of the chosen grid cell
  origin_cell_x_ = static_cast<int16_t>(map_size / 2);
  origin_cell_y_ = origin_cell_x_;

  // Start timer to publish updated map at a regular interval
  map_publish_timer_ = nh_.createTimer(ros::Duration(map_publish_period_), &Mapping::publishMap, this, false, true);
}

Mapping::~Mapping()
{}

void Mapping::spinOnce()
{
  // Publish the map to odom transform
  // Let the origin of the map be in the frame of the ips camera
  // We know the position of the robot base in the odom frame (from tf tree)
  // and the ips camera from the ips message. Hence, we should be able to
  // calculate the map to odom transform

  // Find the transformation from the odom frame to the robot frame
  tf2::Transform odom_to_base_tf;
  // Get position of robot in odom frame from tf tree
  if (tf_buffer_.canTransform(odom_frame_, base_frame_, ros::Time(0)))
  {
    tf2::convert(
      tf_buffer_.lookupTransform(odom_frame_,
                                 base_frame_,
                                 ros::Time(0)).transform,
      odom_to_base_tf);
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0,
                       "Failed to get transform from %s to %s!",
                       base_frame_.c_str(),
                       odom_frame_.c_str());
    return;
  }

  // The transformation from the map to the robot frame is given by the ips/gazebo system
  tf2::Transform map_to_base_tf;
  map_to_base_tf.setOrigin(tf2::Vector3(robot_position_.x,
                                        robot_position_.y,
                                        0.0));
  // TODO(someshdaga): Unsure why the parameters are not in the right order (yaw, pitch, roll)
  //                   Assigning yaw as the first parameter seems to produce a roll
  map_to_base_tf.setRotation(tf2::Quaternion(0.0,
                                             0.0,
                                             robot_yaw_));


  // Calculate transform from map to odom frame
  tf2::convert(map_to_base_tf * odom_to_base_tf.inverse(), map_to_odom_tf_.transform);

  // Set the header time for the transform
  map_to_odom_tf_.header.stamp = ros::Time::now();

  // Broadcast the transform
  tf_broadcaster_.sendTransform(map_to_odom_tf_);

  // Process ROS Callbacks
  ros::spinOnce();
}

void Mapping::spin()
{
  // Process ROS Callbacks at a predefined rate
  // to prevent being bogged down with streams
  // of incoming data
  ros::Rate rate(loop_hz_);
  while (ros::ok())
  {
    rate.sleep();
    spinOnce();
  }
}

void Mapping::drCallback(turtlebot_slam::MappingConfig &config, uint32_t level)
{
  debug_ = config.debug;
  skip_n_scans_ = config.skip_n_scans;
  hit_threshold_ = config.hit_threshold;
  close_hit_occupancy_probability_ = config.close_hit_occupancy_probability;
  far_hit_occupancy_probability_ = config.far_hit_occupancy_probability;
  unknown_occupancy_odds_buffer_ = config.occupancy_odds_buffer;
  saturation_log_odds_occupied_ = config.saturation_occupied;
  saturation_log_odds_free_ = config.saturation_free;
}

void Mapping::updateMap(geometry_msgs::Point scan_point)
{
  // Get the cell position of the robot
  std::pair<int16_t, int16_t> robot_cell = worldToMap(robot_position_.x,
                                                      robot_position_.y);

  // Get the cell position of the hit point of the ray
  std::pair<int16_t, int16_t> ray_cell = worldToMap(scan_point.x, scan_point.y);

  // Check whether the robot is within the bounds of the map
  std::pair<bool, uint8_t> robot_in_map =
    inMapBounds(robot_cell.first, robot_cell.second);

  // Check whether the point of reflection is within the bounds of the map
  std::pair<bool, uint8_t> ray_in_map =
    inMapBounds(ray_cell.first, ray_cell.second);

  // If the bounds of the map are exceeded, adjust the boundaries of the map
  // Note that if the robot and the scan hit point are both within the map, it
  // is enough to guarantee that all intermediate cells will also be within the map
  // due to the nature of the expansion of the map boundaries
  while (!robot_in_map.first || !ray_in_map.first)
  {
    // Adjust the map boundaries based on the cell that is violating the boundaries
    adjustMapBounds(!robot_in_map.first? robot_in_map.second : ray_in_map.second);

    // Get updated cell positions for the robot and ray cells after the boundary updates
    robot_cell = worldToMap(robot_position_.x, robot_position_.y);
    ray_cell = worldToMap(scan_point.x, scan_point.y);

    // Check whether the robot is within the bounds of the map
    robot_in_map =
      inMapBounds(robot_cell.first, robot_cell.second);

    // Check whether the point of reflection is within the bounds of the map
    ray_in_map =
      inMapBounds(ray_cell.first, ray_cell.second);
  }

  // Use Bresenham's Algorithm to determine which cells the ray has travelled through
  int16_t slope_error;  // Keeps track of accumulated error in the slope
  int16_t minor_value;  // The value of the smaller variable
  int16_t dx = ray_cell.first - robot_cell.first;    // Change in x
  int16_t dy = ray_cell.second - robot_cell.second;  // Change in y
  int16_t x_sign = dx >= 0? 1 : -1;                  // The sign of x change
  int16_t y_sign = dy >= 0? 1 : -1;                  // The sign of y change
  bool x_more_y = (abs(dx) >= abs(dy));              // Change in x bigger than y
  int16_t x_cell;
  int16_t y_cell;
  std::vector<std::pair<int16_t, int16_t>> bresenham_pts;  // For debugging purposes

  // To avoid hard coding the different variants for each octant of Bresenham's algorithm
  // create a vector of parameters that will allow us to get away with just using a single
  // algorithm
  std::vector<int16_t> parameters
  {
    x_more_y? x_sign : y_sign,                      // Sign of change in major quantity, 0
    x_more_y? y_sign : x_sign,                      // Sign of change in minor quantity, 1
    x_more_y? dx : dy,                              // Major quantity size, 2
    x_more_y? dy : dx,                              // Minor quantity size, 3
    x_more_y? robot_cell.first : robot_cell.second, // Major quantity start value, 4
    x_more_y? robot_cell.second : robot_cell.first, // Minor quantity start val, 5
    ((x_more_y == (dx >= dy)) == (std::signbit(dx) == std::signbit(dy)))? static_cast<int16_t>(1) :
                                                                          static_cast<int16_t>(-1), // Slope check sign, 6
    (std::signbit(dx) == std::signbit(dy))? static_cast<int16_t>(1) : static_cast<int16_t>(-1)      // Slope, 7
  };

  slope_error = 2*parameters[3] - parameters[7]*parameters[2];
  minor_value = parameters[5];
  for (int i = parameters[4];
        parameters[0]*i < parameters[0]*(parameters[4] + parameters[2]);
        i += parameters[0])
  {
    // If slope threshold condition is met
    // Increment/Decrement the smaller variable and
    // reset the slope error
    if (parameters[6]*slope_error >= 0)
    {
      minor_value += parameters[1];
      slope_error -= parameters[7]*(2*parameters[2]);
    }

    // Adjust the slope error
    slope_error += 2*parameters[3];

    // Assign the x and y cell indexes
    x_cell = (x_more_y)? i : minor_value;
    y_cell = (x_more_y)? minor_value : i;

    // Add cell to vector of bresenham points for visualization
    bresenham_pts.push_back(std::pair<int16_t, int16_t>(x_cell, y_cell));

    // Update the log-odds for the given cell
    updateLogOdds(x_cell,
                  y_cell,
                  ray_cell.first - x_cell,
                  ray_cell.second - y_cell);
  }

  // If debugging is enabled, publish visualization markers for the
  // Bresenham scan lines
  if (debug_)
    drawBresenham(bresenham_pts);
}

void Mapping::drawBresenham(std::vector<std::pair<int16_t, int16_t>> coords)
{
  visualization_msgs::Marker bresenham_marker;
  bresenham_marker.header.stamp = ros::Time::now();
  bresenham_marker.header.frame_id = map_frame_;
  bresenham_marker.id = 0;
  bresenham_marker.type = visualization_msgs::Marker::LINE_STRIP;
  bresenham_marker.action = visualization_msgs::Marker::ADD;
  bresenham_marker.lifetime = ros::Duration(3.0);
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.a = 0.5;
  bresenham_marker.scale.x = 0.025;
  bresenham_marker.color = color;
  for (std::pair<int16_t, int16_t> coord : coords)
  {
    std::pair<double, double> world_coords = mapToWorld(coord.first, coord.second);
    geometry_msgs::Point world_point;
    world_point.x = world_coords.first;
    world_point.y = world_coords.second;
    bresenham_marker.points.push_back(world_point);
    bresenham_marker.colors.push_back(color);
  }
  viz_pub_.publish(bresenham_marker);
}

void Mapping::updateLogOdds(int16_t x_cell, int16_t y_cell, int16_t dx_hit, int16_t dy_hit)
{
  // Use the inverse sensor model to get the probability that the cell is occupied or free
  // based on the distance to the hit point (calculate in cells to avoid floating point operations)
  uint16_t distance_2 = std::pow(dx_hit, 2) + std::pow(dy_hit, 2);
  double probability_occupied = 0.0;

  // High change of being occupied
  if (distance_2 <= hit_threshold_)
    probability_occupied = close_hit_occupancy_probability_;
  // Low change of being occupied
  else
    probability_occupied = far_hit_occupancy_probability_;

  double log_odds_change = std::log(probability_occupied / (1 - probability_occupied));
  // Update the log-odds of the cell based on the prior log-odds and the inverse sensor model
  grid_map_(x_cell, y_cell) += (log_odds_change > 0? std::min(log_odds_change, saturation_log_odds_occupied_) :
                                                     std::max(log_odds_change, -1 * saturation_log_odds_free_));
}

int8_t Mapping::logOddsToOccupancy(double x)
{
  if (abs(x) <= unknown_occupancy_odds_buffer_)
    return static_cast<int8_t>(-1);
  else if (x < 0)
    return static_cast<int8_t>(0);
  else
    return static_cast<int8_t>(100);
}

void Mapping::publishMap(const ros::TimerEvent& e)
{
  // Create an occupancy grid message
  nav_msgs::OccupancyGrid map_msg;

  // Fill in the metadata of the occupancy grid
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = map_frame_;
  map_msg.info.resolution = map_resolution_;
  map_msg.info.width = static_cast<uint32_t>(grid_map_.rows());
  map_msg.info.height = static_cast<uint32_t>(grid_map_.cols());

  // Determine the world coordinates of the origin (top-left corner)
  // of the map
  std::pair<double, double> origin_position = mapToWorld(0,0);
  // Orientation for the map is not important, set rpy to 0
  tf2::Quaternion origin_orientation;
  origin_orientation.setRPY(0,0,0);

  // Create a Pose message to store the position and orientation
  // of the map origin
  geometry_msgs::Pose origin_pose;
  origin_pose.position.x = origin_position.first;
  origin_pose.position.y = origin_position.second;
  origin_pose.position.z = 0.0;
  tf2::convert(origin_orientation, origin_pose.orientation);

  // Set the pose of the origin of the map
  map_msg.info.origin = origin_pose;

  // Convert the log-odds values to occupancy grid values
  Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> grid =
    grid_map_.unaryExpr(std::bind(&Mapping::logOddsToOccupancy, this, std::placeholders::_1));
  // Fill in the data for the occupancy grid message
  map_msg.data = std::vector<int8_t>(grid.data(), grid.data() + grid.size());

  // Publish the occupancy grid
  map_pub_.publish(map_msg);
}

void Mapping::ipsCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& pose)
{
  robot_position_.x = pose->pose.pose.position.x;
  robot_position_.y = pose->pose.pose.position.y;
  robot_yaw_ = tf2::getYaw(pose->pose.pose.orientation);
}

void Mapping::poseCallback(const gazebo_msgs::ModelStatesConstPtr& states)
{
  // Find which index in the list of models the robot is
  auto it = std::find(states->name.begin(), states->name.end(), robot_name_);

  // Ensure a match if found
  if (it != states->name.end())
  {
    uint8_t idx = std::distance(states->name.begin(), it);
    robot_position_ = states->pose[idx].position;
  }
  else
    ROS_FATAL("[MAPPING] No model '%s' in Gazebo!!!", robot_name_.c_str());
}

void Mapping::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  // Note: Scans are obtained in the camera_depth_frame
  //       Assume the frame is the same as the robot frame (impossible to tell
  //       since we don't know how the ips frame relates to the robot/camera frame)
  //       The frames for the robot and camera frame should be close enough to be considered
  //       the same
  // We need to process one ray at a time
  for (uint16_t scan_idx=0; scan_idx < scan->ranges.size(); scan_idx += skip_n_scans_ + 1)
  {
    // Discard range values outside the reported measuring range of the range sensor
    if (std::isnan(scan->ranges[scan_idx]) ||
        scan->ranges[scan_idx] < scan->range_min ||
        scan->ranges[scan_idx] > scan->range_max)
      continue;

    // Get pose of range reading in the frame of the map (ips/gazebo)
    geometry_msgs::Point scan_point;
    double theta = scan->angle_min + scan_idx * scan->angle_increment;
    scan_point.x = robot_position_.x + scan->ranges[scan_idx] *
                                                   cos(robot_yaw_ + theta);
    scan_point.y = robot_position_.y + scan->ranges[scan_idx] *
                                                   sin(robot_yaw_ + theta);

    // Update the log-odds for all cells traversed by this scan point
    updateMap(scan_point);
  }
}

void Mapping::adjustMapBounds(uint8_t violations, double expand)
{
  // Determine how many cells to expand
  int16_t expand_cells = ceil(expand / map_resolution_);

  // Determine the new size of the map in the x-direction
  int16_t new_x_size = grid_map_.rows() +
                       (violations & Mapping::Boundary::RIGHT? expand_cells : 0) +
                       (violations & Mapping::Boundary::LEFT?  expand_cells : 0);
  
  // Determine the new size of the map in the y-direction
  int16_t new_y_size = grid_map_.cols() +
                       (violations & Mapping::Boundary::TOP?    expand_cells : 0) +
                       (violations & Mapping::Boundary::BOTTOM? expand_cells : 0);
  
  // Make note of the current origin cell indexes
  uint16_t old_origin_cell_x = origin_cell_x_;
  uint16_t old_origin_cell_y = origin_cell_y_;

  // If we need to expand the matrix to the left,
  // change the x index of the origin cell
  if (violations & Mapping::Boundary::LEFT)
    origin_cell_x_ += expand_cells;
  
  // If we need to expand the matrix on the upper size,
  // change the y index of the origin cell
  if (violations & Mapping::Boundary::BOTTOM)
    origin_cell_y_ += expand_cells;

  if (violations)
  {
    // Initialize the matrix of zeros, with size equal to that of the desired map
    Eigen::MatrixXd new_map = Eigen::MatrixXd::Zero(new_x_size, new_y_size);

    // Move the contents of the current map into this new map
    new_map.block(origin_cell_x_ - old_origin_cell_x,
                  origin_cell_y_ - old_origin_cell_y,
                  grid_map_.rows(),
                  grid_map_.cols()) = grid_map_;

    // Move the new map into the map variable
    grid_map_ = std::move(new_map);
  }
}

std::pair<bool, uint8_t> Mapping::inMapBounds(double x, double y)
{
  std::pair<int16_t, int16_t> cell_position =
    worldToMap(x,y);
  
  return inMapBounds(cell_position.first, cell_position.second);
}

std::pair<bool, uint8_t> Mapping::inMapBounds(int16_t cell_x, int16_t cell_y)
{
  // If the cell is within the bounds of the grid map matrix
  // then all is good
  if (cell_x >= 0 &&
      cell_y >= 0 &&
      cell_x < grid_map_.rows() &&
      cell_y < grid_map_.cols())
    return std::pair<bool, uint8_t>(true, 0);
  else
  {
    // Get the violated boundaries
    uint8_t violations = 0;
    if (cell_x < 0)
      violations |= Mapping::Boundary::LEFT;
    else if (cell_x >= grid_map_.rows())
      violations |= Boundary::RIGHT;
    
    if (cell_y < 0)
      violations |= Mapping::Boundary::BOTTOM;
    else if (cell_y >= grid_map_.cols())
      violations |= Mapping::Boundary::TOP;

    return std::pair<bool, uint8_t>(false, violations);
  }
}

std::pair<double, double> Mapping::mapToWorld(int16_t cell_x, int16_t cell_y)
{
  // Technically, it's not possible to go from a discretized to a
  // continous representation of the space
  // We merely return the world position at the center of the requested
  // grid cell
  double x = (cell_x - origin_cell_x_) * map_resolution_;
  double y = (cell_y - origin_cell_y_) * map_resolution_;
  return std::pair<double, double>(x, y);
}

std::pair<int16_t, int16_t> Mapping::worldToMap(double x, double y)
{
  int16_t x_cell = round(x / map_resolution_) + origin_cell_x_;
  int16_t y_cell = round(y / map_resolution_) + origin_cell_y_;
  return std::pair<int16_t, int16_t>(x_cell, y_cell);
}

}
}
