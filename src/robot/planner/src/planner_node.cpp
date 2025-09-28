#include "planner_node.hpp"

using std::placeholders::_1;

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  this->declare_parameter("foo", 0);
  // Subscribers
  map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

// g cost = euclidian distance from starting node
double PlannerNode::g_score(double x1, double x2, double y1, double y2){
  double x_diff = x2 - x1;
  double y_diff = y2 - y1;
  double distance = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));

  return distance;
}

// h cost is distance to end node
double PlannerNode::h_score(double xx, double yy){
  double x_diff = goal_.point.x - xx;
  double y_diff = goal_.point.y - yy;
  double distance = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));

  return distance;
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    RCLCPP_INFO(this->get_logger(), "Replanning because new map received");
      planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "goal received");
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "Planning Started: Goal Position Received");
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      if (goalReached()) {
          RCLCPP_INFO(this->get_logger(), "Goal reached!");
          state_ = State::WAITING_FOR_GOAL;
      } else {
          RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
          planPath();
      }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::planPath() {
  RCLCPP_INFO(this->get_logger(), "Planning Path");
  //Beginning of the A* algorithm

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
  std::unordered_set<CellIndex, CellIndexHash> closed_list;

  cell_map.clear();

  //Start by placing the start position in the open list as well as in the hash table
  CellIndex start_idx(robot_pose_.position.x, robot_pose_.position.y);
  double g_start = 0;
  double h_start = h_score(robot_pose_.position.x, robot_pose_.position.y);
  double f_start = g_start + h_start;

  AStarNode start_node(start_idx, f_start, 0.0, h_start, start_idx);
  open_list.push(start_node);
  cell_map[start_idx] = start_node;

  CellIndex target(goal_.point.x, goal_.point.y);

  AStarNode current;
  while(!open_list.empty()){
    
    //when you pop from a priority queue it will automatically
    //remove the element with the lowest fscore
    current = open_list.top();
    open_list.pop();
    closed_list.insert(current.index);

    if(current.index == target){
      //exits loop
      break;
    }

    std::vector<std::pair<int, int>> offsets = {
      { 0,  1},  // up
      { 1,  0},  // right
      { 0, -1},  // down
      {-1,  0},  // left
      { 1,  1},  // up right
      { 1, -1},  // up left
      {-1, -1},  // down left
      {-1, 1}    //down right
    };

    for (const auto& offset : offsets) {
      int dx = offset.first;
      int dy = offset.second;
      int x_pos = current.index.x + dx;
      int y_pos = current.index.y + dy;
      // Is the element within the bounds
      if( (abs(x_pos) > (static_cast<int>(current_map_.info.width) / 2)) || (abs(y_pos) > (static_cast<int>(current_map_.info.height) / 2))){
        continue;
      }
      //neighbour cost
      int neighbour_index = y_pos * current_map_.info.width + x_pos;
      double neighbour_cost = static_cast<double>(current_map_.data[neighbour_index]);
      if(neighbour_cost > 20){
        continue;
      }
      // Is the element already in the closed list
      CellIndex neighbour_cell_index(x_pos, y_pos);
      if(closed_list.count(neighbour_cell_index)){
        continue;
      }

      //The cost to move (either 1.0 or 1.4)
      double cost_to_move = 0;
      if(abs(offset.first) == 1 && abs(offset.second) == 1){
        cost_to_move = 1.4;
      }
      else{
        cost_to_move = 1.0;
      }

      double g_score = cost_to_move + current.g_score;

      //If it is not already in the hash table, or if it was but now has a lower g score.
      auto neighbour_node = cell_map.find(neighbour_cell_index);
      if(neighbour_node == cell_map.end() || g_score < neighbour_node->second.g_score){
        double h = h_score(x_pos, y_pos);
        double f = g_score + h;

        AStarNode neighbour_a_star(neighbour_cell_index, f, g_score, h, current.index);

        cell_map[neighbour_cell_index] = neighbour_a_star;
        open_list.push(neighbour_a_star);
      }
    }



  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  nav_msgs::msg::Path path;

  //start from the target (which should be current)
  //since we set the parent of the start node to itsself, we know that when
  //current index = parent index, we have reached the final node
  RCLCPP_INFO(this->get_logger(), "Creating Path");
  while(!(current.index == current.parent)){
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "sim_world";
    pose.header.stamp = this->get_clock()->now();
    // Scale by resolution then add to the origin
    pose.pose.position.x = current.index.x * 0.1 + current_map_.info.origin.position.x;
    pose.pose.position.y = current.index.y * 0.1 + current_map_.info.origin.position.y;
    pose.pose.position.z = 0.0;
    //Use insert to place elements at the start of the vector:
    poses.insert(poses.begin(), pose);
    current = cell_map[current.parent];
  }

  //publish the path:
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "path";
  path.poses = poses;
  path_publisher_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
