#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), distance_threshold(1.5), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  last_x = 0.0;
  last_y = 0.0;
  last_yaw = 0.0;

  g_resolution = 0.1; // meters per cell
  g_width = 400; // cells
  g_height = 400; // cells
  g_origin_x = -20.0; // meters
  g_origin_y = -20.0; // meters

  global_map_.assign(g_width * g_height, 0); // Initialize all cells to free space (0)

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "costmap", 
    10,
    std::bind(&MapMemoryNode::mapCallback, this, std::placeholders::_1) 
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this)
  );
}

void MapMemoryNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  curr_x = x;
  curr_y = y;
  curr_yaw = std::atan2(2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y), 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));

  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  if (distance >= distance_threshold) {
    last_x = x;
    last_y = y;
    auto q = msg->pose.pose.orientation;
    last_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    publishGlobalMap();
    should_update_map_ = false;
  }
}

void MapMemoryNode::integrateCostmap() {
  // Loop through whole global map
  for (int gy = 0; gy < g_height; ++gy) {
    for (int gx = 0; gx < g_width; ++gx) {
      // Convert global map cell to world coordinates
      // Note that +0.5 is added to get the center of the cell
      double world_x = g_origin_x + (gx + 0.5) * g_resolution;
      double world_y = g_origin_y + (gy + 0.5) * g_resolution;

      // Convert world coordinates to robot frame coordinates
      double dx = world_x - curr_x;
      double dy = world_y - curr_y;
      double robot_x = std::cos(curr_yaw) * dx + std::sin(curr_yaw) * dy;
      double robot_y = -std::sin(curr_yaw) * dx + std::cos(curr_yaw) * dy;

      // Convert robot frame coordinates to costmap grid coordinates
      int costmap_x = static_cast<int>(std::floor((robot_x - latest_costmap_.info.origin.position.x) / latest_costmap_.info.resolution));
      int costmap_y = static_cast<int>(std::floor((robot_y - latest_costmap_.info.origin.position.y) / latest_costmap_.info.resolution));

      // Check if costmap coordinates are within bounds
      if (costmap_x < 0 || costmap_x >= static_cast<int>(latest_costmap_.info.width) ||
          costmap_y < 0 || costmap_y >= static_cast<int>(latest_costmap_.info.height)) {
        continue; // Skip if out of bounds
      }

      int8_t value = latest_costmap_.data[costmap_y * latest_costmap_.info.width + costmap_x];
      global_map_[gy * g_width + gx] = std::max(value, global_map_[gy * g_width + gx]); // Keep the highest cost, pessimistic (obstacles are permanent)
    }
  }
}

void MapMemoryNode::publishGlobalMap() {
  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.stamp = latest_costmap_.header.stamp;
  map_msg.header.frame_id = "sim_world";
  map_msg.info.resolution = g_resolution;
  map_msg.info.width = g_width;
  map_msg.info.height = g_height;
  map_msg.info.origin.position.x = g_origin_x;
  map_msg.info.origin.position.y = g_origin_y;
  map_msg.data = global_map_;
  map_pub_->publish(map_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
