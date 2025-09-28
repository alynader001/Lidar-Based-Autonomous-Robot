#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 0.2;  // Lookahead distance
  goal_tolerance_ = 0.2;     // Distance to consider the goal reached
  linear_speed_ = 0.1;       // Constant forward speed

  // Subscribers and Publishers
  path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_) {
      return;
  }

  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
      geometry_msgs::msg::Twist stop_vel;
      stop_vel.linear.x = 0.0;
      stop_vel.angular.z = 0.0;
      cmd_vel_publisher_->publish(stop_vel);
      return;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_publisher_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if(!current_path_ || current_path_->poses.empty() || !robot_odom_){
    return std::nullopt;
  }

  double robot_x = robot_odom_->pose.pose.position.x;
  double robot_y = robot_odom_->pose.pose.position.y;

  const geometry_msgs::msg::PoseStamped &target_position = current_path_->poses.back();

  double x_to_target = robot_x - target_position.pose.position.x;
  double y_to_target = robot_y - target_position.pose.position.y;
  double distance_to_target = std::sqrt(x_to_target * x_to_target + y_to_target * y_to_target);

  if(distance_to_target <= goal_tolerance_){
    return std::nullopt;
  }

  for(const auto& pose : current_path_->poses){
    double dx = pose.pose.position.x - robot_x;
    double dy = pose.pose.position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance >= lookahead_distance_) {
      return pose;
    }
  }

  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  double sin_heading = 2.0 * (robot_odom_->pose.pose.orientation.w * robot_odom_->pose.pose.orientation.z + robot_odom_->pose.pose.orientation.x * robot_odom_->pose.pose.orientation.y);
  double cos_heading = 1.0 - 2.0 * (robot_odom_->pose.pose.orientation.y * robot_odom_->pose.pose.orientation.y + robot_odom_->pose.pose.orientation.z * robot_odom_->pose.pose.orientation.z);
  double robot_heading = std::atan2(sin_heading, cos_heading);
  
  double dx = target.pose.position.x - robot_odom_->pose.pose.position.x;
  double dy = target.pose.position.y - robot_odom_->pose.pose.position.y;

  double target_angle = std::atan2(dy, dx);
  double target_distance = (dx * dx + dy * dy);

  double angle_error = std::fmod(target_angle - robot_heading + M_PI, 2.0 * M_PI) - M_PI;

  cmd_vel.linear.x = linear_speed_ * target_distance;
  cmd_vel.angular.z = angle_error;

  return cmd_vel;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
