#define _USE_MATH_DEFINES
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

#include "wheelchair_interfaces/action/move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

class MovementHandler : public rclcpp::Node {
public:
  using Movement = wheelchair_interfaces::action::Move;
  using GoalHandleMovement = rclcpp_action::ClientGoalHandle<Movement>;

  explicit MovementHandler() : Node("handler"){
    this->subscription_ = this->create_subscription<std_msgs::msg::String>(
        "speech", 10, std::bind(&MovementHandler::callback, this, std::placeholders::_1));
    this->client_ = rclcpp_action::create_client<Movement>(this, "movement");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp_action::Client<Movement>::SharedPtr client_;

  void callback(const std_msgs::msg::String::SharedPtr msg){
    // split the message into words
    std::istringstream iss(msg->data);
    std::vector<std::string> words(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());

    // check if the first word is a valid action
    if (words[0] == "forward"){
      // try to send the goal
      move(words[0]);
    } else if (words[0] == "backward"){
      move(words[0]);
    } else if (words[0] == "rotate"){
      try{
        if (words[1] == "left" || words[1] == "counterclockwise"){
          float angle = std::stof(words[2]) * M_PI / 180;
          RCLCPP_INFO(this->get_logger(), "Angle: %f", angle);
          move(words[0], angle);
        } else if (words[1] == "right" || words[1] == "clockwise"){
          float angle = -std::stof(words[2]) * M_PI / 180;
          RCLCPP_INFO(this->get_logger(), "Angle: %f", angle);
          move(words[0], angle);
        } else {
          RCLCPP_INFO(this->get_logger(), "Invalid action");
        }
      } catch (std::exception &e){
        RCLCPP_INFO(this->get_logger(), "Invalid action");
      }
    } else if (words[0] == "stop"){
      move(words[0]);
    } else if (words[0] == "cancel") {
      cancel();
    } else {
      RCLCPP_INFO(this->get_logger(), "Invalid action");
    }
  }

  void cancel() {
    auto cancel_future = client_->async_cancel_all_goals();
    std::thread([this, cancel_future]() {
      if (cancel_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal");
      }
    }).detach();
  }

  void move(std::string type, float angle = 0.0){
    if (!client_->wait_for_action_server(std::chrono::seconds(10))){
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = Movement::Goal();
    goal_msg.movement = type;
    goal_msg.angle = angle;

    auto send_goal_options = rclcpp_action::Client<Movement>::SendGoalOptions();
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovementHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}