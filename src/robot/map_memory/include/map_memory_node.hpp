#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"
#include "rclcpp/timer.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    void integrateCostmap();

    nav_msgs::msg::OccupancyGrid latest_costmap_;

    void publishGlobalMap();

    std::vector<int8_t> global_map_;
    double g_resolution; // in meters (should besame as costmap resolution)
    int g_width, g_height; // in cells
    double g_origin_x, g_origin_y; // in meters

    double curr_x, curr_y, curr_yaw;
    double last_x, last_y, last_yaw;
    const double distance_threshold; // in meters
    bool costmap_updated_ = false;

    bool should_update_map_ = false;
};

#endif 
