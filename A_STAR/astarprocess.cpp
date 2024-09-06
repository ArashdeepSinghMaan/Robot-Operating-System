#include "Astar.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/SimpleActionClient.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace pathplanning {

class Astar {
 public:
  // ... existing member functions ...

  void initRos(ros::NodeHandle& nh);

 private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
  void sendGoal(const Point& goal_point);

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Publisher path_pub_;  // Optional: To publish the planned path
  tf::TransformListener tf_listener_;
  geometry_msgs::PoseStamped current_pose_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

  // Helper functions for converting between ROS and A* representation
  Point rosPointToMap(const geometry_msgs::Point& ros_point);
  geometry_msgs::Point mapPointToRos(const Point& map_point);
};

void Astar::initRos(ros::NodeHandle& nh) {
  nh_ = nh;
  map_sub_ = nh_.subscribe("/map", 10, &Astar::mapCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("planned_path", 10); // Optional
  tf_listener_.setBuffer(ros::Duration(10.0));
  move_base_client_ = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
      "/move_base", true);

  // Get initial robot pose
  geometry_msgs::PoseStamped robot_pose;
  tf_listener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
  tf_listener_.lookupTransform("/map", "/base_link", ros::Time(0), robot_pose);
  current_pose_ = robot_pose;
}

void Astar::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  // Update internal map representation with received occupancy grid
  // (Convert ROS data format to your A* map format)
  Map = convertRosMapToAstarMap(map->data, map->info.width, map->info.height);
}

void Astar::sendGoal(const Point& goal_point) {
  // Convert A* point to ROS point and create a navigation goal
  geometry_msgs::PoseStamped goal_pose = mapPointToRos(goal_point);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = goal_pose;

  // Send the goal to move_base action server
  move_base_client_.sendGoal(goal);
  move_base_client_.waitForResult();

  if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Goal reached successfully!");
  } else {
    ROS_ERROR("Failed to reach goal!");
  }
}

// Helper functions for ROS-A* data conversion (implementation omitted)
Point Astar::rosPointToMap(const geometry_msgs::Point& ros_point);
geometry_msgs::Point Astar::mapPointToRos(const Point& map_point);

};

} // namespace pathplanning

