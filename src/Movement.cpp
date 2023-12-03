#include <functional>
#include <memory>
#include <thread>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <array>

#include "wheelchair_interfaces/action/move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "wheelchair/visibility_control.h"

namespace wheelchair{

class MovementActionServer : public rclcpp::Node {
public:
  using Movement = wheelchair_interfaces::action::Move;
  using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;

  WHEELCHAIR_PUBLIC
  explicit MovementActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("movement_action_server", options) {
    this->action_server_ = rclcpp_action::create_server<Movement>(
        this,
        "movement",
        std::bind(&MovementActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MovementActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MovementActionServer::handle_accepted, this, std::placeholders::_1)
    );
    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("model/wheelchair/cmd_vel", 10);
    this->subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "model/wheelchair/odometry", 10, std::bind(&MovementActionServer::callback, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<Movement>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

  // store the last angle and the remaining rotation to be performed
  float remaining_rotation = 0.0;
  float last_angle = 0.0;
  std::string movement = "";

  static std::array<std::string, 4> accepted_actions;
  static float allowed_error;

  void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    float delta_angle = msg->pose.pose.orientation.z - this->last_angle;
    if (delta_angle > 0) {
      this->remaining_rotation -= delta_angle;
    } else {
      this->remaining_rotation += delta_angle;
    }
    this->last_angle = msg->pose.pose.orientation.z;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Movement::Goal> goal) {

    // reject if the movement is not part of the accepted actions
    if (std::find(MovementActionServer::accepted_actions.begin(), MovementActionServer::accepted_actions.end(), goal->movement) == MovementActionServer::accepted_actions.end()) {
        return rclcpp_action::GoalResponse::REJECT;
    } else {
        this->movement = goal->movement;
        this->remaining_rotation = (float)goal->angle;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMovement> goal_handle) {
      // reset the remaining rotation and the last angle
      this->remaining_rotation = 0.0;
      this->last_angle = 0.0;
      this->movement = "";

      // stop the wheelchair
      geometry_msgs::msg::Twist msg;
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      this->publisher_->publish(msg);

      RCLCPP_INFO(this->get_logger(), "Goal canceled");

      return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle) {
    std::thread{std::bind(&MovementActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMovement> goal_handle) {
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Movement::Feedback>();
    auto & remaining_angle = feedback->angle;
    auto result = std::make_shared<Movement::Result>();

    // handle forward and backward
    if (this->movement == "forward"){
      geometry_msgs::msg::Twist msg;
      msg.linear.x = 5.0;
      msg.angular.z = 0.0;
      this->publisher_->publish(msg);
    } else if (this->movement == "backward") {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = -5.0;
      msg.angular.z = 0.0;
      this->publisher_->publish(msg);
    } else if (this->movement == "stop") {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      this->publisher_->publish(msg);
    } else {
      while(std::abs(this->remaining_rotation) > MovementActionServer::allowed_error) {
        RCLCPP_INFO(this->get_logger(), "Remaining rotation: %f", this->remaining_rotation);
        if (!rclcpp::ok()) {
          return;
        }
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = this->remaining_rotation > 0 ? 0.5 : -0.5;
        this->publisher_->publish(msg);

        remaining_angle = this->remaining_rotation;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
      }

      geometry_msgs::msg::Twist msg;
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      this->publisher_->publish(msg);
    }

    if (rclcpp::ok()){
      result->angle = this->remaining_rotation;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Succeeded");
      this->remaining_rotation = 0.0;
      this->last_angle = 0.0;
      this->movement = "";
    }
  }
};

std::array<std::string, 4> MovementActionServer::accepted_actions = {"forward", "backward", "rotate", "stop"};
float MovementActionServer::allowed_error = 0.1;
}


RCLCPP_COMPONENTS_REGISTER_NODE(wheelchair::MovementActionServer)