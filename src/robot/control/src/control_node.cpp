#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this)
  );

}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
}

void ControlNode::controlLoop() {
  if (current_path_.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path received yet.");
    return;
  }

  if (current_odom_.header.stamp.sec == 0 && current_odom_.header.stamp.nanosec == 0) {
    RCLCPP_WARN(this->get_logger(), "No odometry received yet.");
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    RCLCPP_WARN(this->get_logger(), "No lookahead point found.");
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::Point> ControlNode::findLookaheadPoint() {
  const auto & robot_position = current_odom_.pose.pose.position;
  // Along the path, find the point closest to the robot first
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_index = 0;
  for (size_t i = 0; i < current_path_.poses.size(); ++i) {
    const auto & path_point = current_path_.poses[i].pose.position;
    double distance = computeDistance(robot_position, path_point);
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }

  // Now that we have the closest point, find the lookahead point
  size_t target = current_path_.poses.size() - 1; // Default to last point if no lookahead point is found at this moement
  for (size_t i = closest_index; i < current_path_.poses.size(); ++i) {
    const auto & path_point = current_path_.poses[i].pose.position;
    double distance = computeDistance(robot_position, path_point);
    if (distance >= lookahead_distance_) {
      target = i;
      break;
    }
  }

  return current_path_.poses[target].pose.position;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::Point & lookahead_point) {
  const auto & robot_position = current_odom_.pose.pose.position;
  const auto & robot_orientation = current_odom_.pose.pose.orientation;

  double yaw = extractYaw(robot_orientation);

  double dx = lookahead_point.x - robot_position.x;
  double dy = lookahead_point.y - robot_position.y;

  double target_yaw = std::atan2(dy, dx);
  double yaw_error = target_yaw - yaw;

  // Normalize yaw error to [-pi, pi]
  while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = yaw_error; // Simple P controller for angular velocity

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion & q) {
  // Convert quaternion to yaw
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
