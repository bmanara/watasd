#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;

    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::Odometry current_odom_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();

    std::optional<geometry_msgs::msg::Point> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::Point & lookahead_point);
    double computeDistance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);
    double extractYaw(const geometry_msgs::msg::Quaternion & q);
};

#endif
