#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdometrySubscriber : public rclcpp::Node {
public:
  OdometrySubscriber() : Node("odometer") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/model/wheelchair/odometry", 10, std::bind(&OdometrySubscriber::callback, this, std::placeholders::_1));
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Orientation: %f", msg->pose.pose.orientation.z);
    RCLCPP_INFO(this->get_logger(), "Linear Velocity: %f", msg->twist.twist.linear.x);
    RCLCPP_INFO(this->get_logger(), "Angular Velocity: %f", msg->twist.twist.angular.z);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySubscriber>());
  rclcpp::shutdown();
  return 0;
}
