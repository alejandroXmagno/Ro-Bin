#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>
#include <functional>
#include <thread>
#include "custom_interfaces/action/track_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MyActionClient : public rclcpp::Node
{
public:
  using TrackObject = custom_interfaces::action::TrackObject;
  using GoalHandleTrackObject = rclcpp_action::ClientGoalHandle<TrackObject>;

  explicit MyActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("my_action_client", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<TrackObject>(
      this,
      "track_object");

    // Start the main loop in a separate thread
    main_thread_ = std::thread(&MyActionClient::main_loop, this);
  }

  ~MyActionClient()
  {
    if (main_thread_.joinable()) {
      main_thread_.join();
  }
  }

  void main_loop()
  {
    while (rclcpp::ok()) {
      // Get object name from user
      std::string object_name;
      std::cout << "Enter object name to track (or 'q' to quit): ";
      std::getline(std::cin, object_name);
      
      if (object_name == "q") {
        break;
    }

      // Wait for action server
      if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        continue;
      }

      // Send goal
      auto goal_msg = TrackObject::Goal();
      goal_msg.object_name = object_name;

      RCLCPP_INFO(this->get_logger(), "Sending goal to track object: %s", object_name.c_str());

      auto send_goal_options = rclcpp_action::Client<TrackObject>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MyActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&MyActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&MyActionClient::result_callback, this, std::placeholders::_1);
      
      auto future_goal_handle = client_ptr_->async_send_goal(goal_msg, send_goal_options);
      
      // Wait for the goal to complete
      try {
        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
          continue;
        }
        
        // Wait for the result
        auto result_future = client_ptr_->async_get_result(goal_handle);
        result_future.wait();
        
        // After getting the result, wait a moment before asking for the next object
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error while executing goal: %s", e.what());
  }
    }
  }

private:
  rclcpp_action::Client<TrackObject>::SharedPtr client_ptr_;
  std::thread main_thread_;

  void goal_response_callback(const GoalHandleTrackObject::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleTrackObject::SharedPtr,
    const std::shared_ptr<const TrackObject::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->message.c_str());
    std::cout << "Feedback: " << feedback->message << std::endl;
  }

  void result_callback(const GoalHandleTrackObject::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", result.result->message.c_str());
        std::cout << "Goal succeeded: " << result.result->message << std::endl;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted: %s", result.result->message.c_str());
        std::cout << "Goal was aborted: " << result.result->message << std::endl;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled: %s", result.result->message.c_str());
        std::cout << "Goal was canceled: " << result.result->message << std::endl;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        std::cout << "Unknown result code" << std::endl;
        break;
    }
  }
};  // class MyActionClient

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MyActionClient>();
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}