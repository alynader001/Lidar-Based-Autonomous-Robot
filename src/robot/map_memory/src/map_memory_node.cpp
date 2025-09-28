#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), last_x(0.0), last_y(0.0), distance_threshold(3.0) {
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            latest_costmap_ = *msg;

            bool is_empty = std::all_of(msg->data.begin(), msg->data.end(), [](int8_t value) { return value == 0; });
            if (should_update_map_ && !is_empty) {
                updateMap();
            }
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto &orientation = msg->pose.pose.orientation;

            tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
            tf2::Matrix3x3 matrix(quaternion);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);

            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;

            double distance_moved = std::sqrt(
                std::pow(x - last_x, 2) + std::pow(y - last_y, 2));

            if (distance_moved >= distance_threshold) {
                last_x = x;
                last_y = y;
                should_update_map_ = true;
            }
        });

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    global_map_.data.resize(100 * 100, 0);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { publishMap(); });
}

void MapMemoryNode::publishMap() {
    global_map_.header.stamp = this->now();
    global_map_.header.frame_id = "sim_world";
    map_pub_->publish(global_map_);
}

void MapMemoryNode::updateMap() {
  integrateCostmap();

  global_map_.info.resolution = 0.1;
  global_map_.info.width = 100;
  global_map_.info.height = 100;

  const double half_size_res = -100 / 2.0 * 0.1;
  global_map_.info.origin.position.x = half_size_res;
  global_map_.info.origin.position.y = half_size_res;

  should_update_map_ = false;
}

void MapMemoryNode::integrateCostmap() {
  const std::vector<int8_t> &costmap_data = latest_costmap_.data;
  std::vector<int8_t> &global_map_data = global_map_.data;

  double cos_yaw = std::cos(last_y);
  double sin_yaw = std::sin(last_y);

  int local_size = latest_costmap_.info.width;
  double local_origin_x = latest_costmap_.info.origin.position.x;
  double local_origin_y = latest_costmap_.info.origin.position.y;
  double local_resolution = latest_costmap_.info.resolution;

  double global_origin_offset = 100 / -2 * 0.1;
  double resolution_inv = 1.0 / 0.1;

  for (int i = 0; i < local_size; ++i) {
      double local_y = local_origin_y + (i + 0.5) * local_resolution;
      double transformed_y = sin_yaw * local_y;
      double rotated_y = cos_yaw * local_y;

      for (int j = 0; j < local_size; ++j) {
          int occupancy_value = costmap_data[i * local_size + j];
          if (occupancy_value < 0) { continue; }

          double local_x = local_origin_x + (j + 0.5) * local_resolution;
          double global_x = last_x + (cos_yaw * local_x - transformed_y);
          double global_y = last_y + (sin_yaw * local_x + rotated_y);

          int global_map_x = static_cast<int>((global_x - global_origin_offset) * resolution_inv);
          int global_map_y = static_cast<int>((global_y - global_origin_offset) * resolution_inv);

          if (global_map_x >= 0 && global_map_x < 100 && global_map_y >= 0 && global_map_y < 100) {
              int map_index = global_map_y * 100 + global_map_x;
              int8_t &global_cell_value = global_map_data[map_index];
              global_cell_value = std::max(global_cell_value < 0 ? 0 : global_cell_value, occupancy_value);
          }
      }
  }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}