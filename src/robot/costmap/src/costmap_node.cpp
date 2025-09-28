#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"

using std::placeholders::_1;
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>
    ("lidar", 10, std::bind(&CostmapNode::lidarProcessor, this, std::placeholders::_1));

  occupancy_grid_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

}

void CostmapNode::lidarProcessor(const sensor_msgs::msg::LaserScan::SharedPtr scan){
  if(!scan){
    RCLCPP_ERROR(this->get_logger(), "Error: No lidar data received");
  }
  //Initializing the grid
  int length = 100;
  int width = 100;
  double resolution = 0.1; //0.1m
  std::vector<std::vector<int8_t>> matrix(length, std::vector<int8_t>(width, 0));

  //populating the grid with 100s where laser detects objects
  for(int i = 0; i < scan->ranges.size(); ++i){
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      //calculating coordinates
      double x_val = range * std::cos(angle);
      double y_val = range * std::sin(angle);
      //marking obstacls
      int x_pos = 10 * x_val;
      int y_pos = 10 * y_val;
      if(y_pos >= 0 && y_pos < length && x_pos >= 0 && x_pos < width){
        matrix[y_pos][x_pos] = 100;
      }
    }
  }
  //inflate obstacles, using a radius of 0.4m which is 4 squares in a grid
  double inflation_radius = 4;

  for(int i = 0; i < length; ++i){
    for(int j = 0; j < width; ++j){
      if(matrix[i][j] == 100) {

        for(int di = -inflation_radius; di <= inflation_radius; ++di){
          for(int dj = -inflation_radius; dj <= inflation_radius; ++dj){
            
            int ni = i + di;
            int nj = j + dj;

            if(ni >= 0 && ni < length && nj >= 0 && nj < width){
              double distance = std::sqrt(di * di + dj * dj);
              if(distance < inflation_radius){
                double cost = 100 * (1.0 - distance / inflation_radius);
                matrix[ni][nj] = std::min<int8_t>(100, std::max(matrix[ni][nj], static_cast<int8_t>(cost)));
              }
            }
          }
        }
      }
    }
  }

  // //populating the occupancy grid message before publishing
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header = scan->header;
  occupancy_grid.info.resolution = resolution;
  occupancy_grid.info.height = length;
  occupancy_grid.info.width = width;
  //origin at centre
  occupancy_grid.info.origin.position.x = -(width * resolution) / 2.0;
  occupancy_grid.info.origin.position.y = -(length * resolution) / 2.0;
  //flattening the matrix and asigning it to occupancy_grid.data
  occupancy_grid.data.resize(length * width);
  for(int y = 0; y < length; ++y){
    for(int x = 0; x < width; ++x){
      int index = y * width + x;
      occupancy_grid.data[index] = matrix[y][x];
    }
  }
  occupancy_grid_publisher->publish(occupancy_grid);
}
 
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}