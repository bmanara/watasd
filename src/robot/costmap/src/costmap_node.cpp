#include <chrono>
#include <memory>

#include "costmap_node.hpp"
#include "costmap_core.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Create a publisher for the costmap
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);

  // Create a subscription to the lidar topic
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "lidar", 10, std::bind(&CostmapNode::laser_callback, this, std::placeholders::_1));
  
  costmap_ = robot::CostmapCore(this->get_logger());
}

void CostmapNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  this->initializeCostmap();

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double angle = msg->angle_min + i * msg->angle_increment;
    double range = msg->ranges[i];
    if (range < msg->range_max && range > msg->range_min) {
      this->markObstacle(range, angle);
    }
  }

  this->inflateObstacles();
  this->publishCostmap();
}

void CostmapNode::initializeCostmap() {
  this->costmap_.reset();
}

void CostmapNode::markObstacle(double range, double angle) {
  int x_grid, y_grid;
  this->costmap_.convertToGrid(range, angle, x_grid, y_grid);
  this->costmap_.setCell(x_grid, y_grid, 100);
}

void CostmapNode::inflateObstacles() {
  this->costmap_.inflateObstacles();
}

void CostmapNode::publishCostmap() {
  std::vector<int8_t> costmap_data = this->costmap_.getCostmap();
  nav_msgs::msg::OccupancyGrid costmap_msg;
  costmap_msg.header.stamp = this->now();
  costmap_msg.header.frame_id = "map";
  costmap_msg.info.resolution = this->costmap_.resolution_;
  costmap_msg.info.width = this->costmap_.width;
  costmap_msg.info.height = this->costmap_.height;
  costmap_msg.info.origin.position.x = this->costmap_.origin_x_;
  costmap_msg.info.origin.position.y = this->costmap_.origin_y_;
  costmap_msg.data = costmap_data;
  this->costmap_pub_->publish(costmap_msg);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
