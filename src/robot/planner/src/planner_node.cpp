#include "planner_node.hpp"

#include <queue>

PlannerNode::PlannerNode() : Node("planner"), state_(State::WAITING_FOR_GOAL), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  latest_goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Robot reached goal.");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = latest_goal_.point.x - robot_pose_.position.x;
  double dy = latest_goal_.point.y - robot_pose_.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  return distance < goal_tolerance_;
}

bool PlannerNode::pointToCell(
    double point_x, double point_y, 
    double resolution,
    double origin_x, double origin_y,
    int width, int height,
    int & cell_x, int & cell_y) 
{
  cell_x = static_cast<int>((point_x - origin_x) / resolution);
  cell_y = static_cast<int>((point_y - origin_y) / resolution);

  return cell_x >= 0 && cell_x < width && cell_y >= 0 && cell_y < height;
}

double PlannerNode::cellCost(int idx) {
  if (idx < 0 || idx >= static_cast<int>(latest_map_.data.size())) {
    return std::numeric_limits<double>::infinity(); // Out of bounds
  }
  int value = latest_map_.data[idx];
  if (value < 0) {
    return 0; // Unknown cells are treated as free
  }
  return static_cast<double>(value) / 100.0; // Normalize cost to [0,1]
}

double PlannerNode::heuristicCost(int x, int y, int goal_x, int goal_y) {
  // Just use Euclidean distance
  return std::sqrt(std::pow(x - goal_x, 2) +
                   std::pow(y - goal_y, 2));
}

void PlannerNode::planPath() {
  if (!goal_received_ || latest_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: goal not received or map not available.");
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "sim_world";

  // A* planning logic
  const int width = latest_map_.info.width;
  const int height = latest_map_.info.height;
  const int total_cells = width * height;
  const double resolution = latest_map_.info.resolution;

  std::vector<double> g(total_cells, std::numeric_limits<double>::infinity()); // Cost from start to node
  std::vector<int> came_from(total_cells, -1); // To reconstruct path
  std::vector<bool> closed_set(total_cells, false); // Visited nodes

  using Node = std::pair<double, int>; // (f_score, index)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;

  // Convert robot and goal positions to cell indices (map cells)
  int start_x, start_y, goal_x, goal_y;
  if (!pointToCell(robot_pose_.position.x, robot_pose_.position.y, resolution,
                   latest_map_.info.origin.position.x, latest_map_.info.origin.position.y,
                   width, height, start_x, start_y)) {
    RCLCPP_ERROR(this->get_logger(), "Robot position is out of map bounds.");
    return;
  }

  if (!pointToCell(latest_goal_.point.x, latest_goal_.point.y, resolution,
                   latest_map_.info.origin.position.x, latest_map_.info.origin.position.y,
                   width, height, goal_x, goal_y)) {
    RCLCPP_ERROR(this->get_logger(), "Goal position is out of map bounds.");
    return;
  }

  int start_idx = start_y * width + start_x;
  int goal_idx = goal_y * width + goal_x;

  g[start_idx] = 0.0;
  double f_start = heuristicCost(start_x, start_y, goal_x, goal_y);
  open_set.emplace(f_start, start_idx);
  while(!open_set.empty()) {
    Node current_node = open_set.top();
    open_set.pop();

    int current_idx = current_node.second;
    if (current_idx == goal_idx) {
      break;
    }
    if (closed_set[current_idx]) {
      continue; // Skip, already processed
    }

    closed_set[current_idx] = true;

    int current_x = current_idx % width;
    int current_y = current_idx / width;
    for (int dy = -1; dy < 2; ++dy) {
      for (int dx = -1; dx < 2; ++dx) {
        if (dx == 0 && dy == 0) continue;
        int neighbor_x = current_x + dx;
        int neighbor_y = current_y + dy;
        if (neighbor_x < 0 || neighbor_x >= width || neighbor_y < 0 || neighbor_y >= height) continue;

        int neighbor_idx = neighbor_y * width + neighbor_x;
        if (closed_set[neighbor_idx]) continue;

        if (cellCost(neighbor_idx) >= obstacle_threshold_) continue; // Skip obstacles

        double step_cost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0; // Diagonal cost
        double tentative_g = g[current_idx] + step_cost + cellCost(neighbor_idx) * penalty_factor_;

        if (tentative_g < g[neighbor_idx]) { // Found a better path
          came_from[neighbor_idx] = current_idx;
          g[neighbor_idx] = tentative_g;
          double f_score = tentative_g + heuristicCost(neighbor_x, neighbor_y, goal_x, goal_y);
          open_set.emplace(f_score, neighbor_idx);
        }
      }
    }
  }

  // Reconstruct path
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  int current = goal_idx;
  while (current != -1) {
    int x = current % width;
    int y = current / width;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "sim_world";
    pose_stamped.pose.position.x = x * resolution + latest_map_.info.origin.position.x;
    pose_stamped.pose.position.y = y * resolution + latest_map_.info.origin.position.y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0; // No rotation
    poses.push_back(pose_stamped);
    current = came_from[current];
  }

  std::reverse(poses.begin(), poses.end());
  path_msg.poses = poses;

  path_pub_->publish(path_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
