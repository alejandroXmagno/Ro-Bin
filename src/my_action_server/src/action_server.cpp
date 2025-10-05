#include <functional>
#include <memory>
#include <thread>
#include <set>
#include <deque>
#include <numeric>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/track_object.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MyActionServer : public rclcpp::Node
{
public:
  using TrackObject = custom_interfaces::action::TrackObject;
  using GoalHandleTrackObject = rclcpp_action::ServerGoalHandle<TrackObject>;
  explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("my_action_server", options)
  {
    using namespace std::placeholders;
    // RCLCPP_INFO(this->get_logger(), "Creating action server...");
    
    this->action_server_ = rclcpp_action::create_server<TrackObject>(
      this,
      "track_object",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1));
    
    // RCLCPP_INFO(this->get_logger(), "Creating detection subscription...");
    subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detections_output", 10, std::bind(&MyActionServer::detection_callback, this, _1));
    // RCLCPP_INFO(this->get_logger(), "Detection subscription created successfully");

    // RCLCPP_INFO(this->get_logger(), "Creating depth image subscription...");
    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth", 10, std::bind(&MyActionServer::depth_callback, this, _1));
    // RCLCPP_INFO(this->get_logger(), "Depth image subscription created successfully");
    
    // Create publisher for robot control
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Create a timer to control robot movement
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz control rate
      std::bind(&MyActionServer::control_callback, this));
  }
private:
  rclcpp_action::Server<TrackObject>::SharedPtr action_server_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::string current_object_;
  bool object_in_fov_ = false;
  vision_msgs::msg::Detection2DArray::SharedPtr last_detection_;
  sensor_msgs::msg::Image::SharedPtr last_depth_image_;
  std::set<std::string> detected_objects_;
  
  // Buffer for rolling average
  std::deque<double> depth_history_;
  const size_t HISTORY_SIZE = 20;

  // Control parameters
  const double MAX_LINEAR_SPEED = 0.3;  // m/s
  const double MIN_LINEAR_SPEED = 0.05;  // m/s
  const double MAX_ANGULAR_SPEED = 1.0; // rad/s
  const double KP_STEERING = 0.01;      // Proportional gain for steering
  const double KP_SPEED = 0.5;          // Proportional gain for speed
  const double SEARCH_SPEED = 0.5;      // rad/s for searching

  bool is_tracking = false;

  double calculate_weighted_average() {
    if (depth_history_.empty()) return 0.0;
    
    double total_weight = 0.0;
    double weighted_sum = 0.0;
    
    // Calculate weights (linear increase from oldest to newest)
    for (size_t i = 0; i < depth_history_.size(); ++i) {
      double weight = static_cast<double>(i + 1) / depth_history_.size();
      weighted_sum += depth_history_[i] * weight;
      total_weight += weight;
    }
    
    return weighted_sum / total_weight;
  }

  void control_callback()
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();

    if (!is_tracking) {
      twist_msg->angular.z = 0.0;
      twist_msg->linear.x = 0.0;
    } else if (!object_in_fov_) {
      // Object not in view, perform point turn
      twist_msg->angular.z = SEARCH_SPEED;
      twist_msg->linear.x = 0.0;
    } else if (last_detection_ && last_depth_image_) {
      // Find the target object in detections
      for (const auto & detection : last_detection_->detections) {
        for (const auto & detection_result : detection.results) {
          if (detection_result.hypothesis.class_id == current_object_) {
            // Calculate error from center (320 is center of 640x480 image)
            double error = detection.bbox.center.position.x - 320.0;
            
            // Calculate angular velocity using proportional control
            twist_msg->angular.z = -KP_STEERING * error;

            RCLCPP_INFO(this->get_logger(), "Angular velocity: %.3f rad/s", twist_msg->angular.z);
            
            // Calculate linear velocity based on distance
            double weighted_average = calculate_weighted_average();
            if (weighted_average > 0.2) {  // If more than 20cm away
              // Speed increases with distance, but capped
              double target_speed = KP_SPEED * weighted_average;
              twist_msg->linear.x = std::min(std::max(target_speed, MIN_LINEAR_SPEED), MAX_LINEAR_SPEED);
            } else {
              // Stop when close enough
              twist_msg->linear.x = 0.0;
            }
            break;
          }
        }
      }
    }
    
    cmd_vel_publisher_->publish(std::move(twist_msg));
  }

  void print_detected_objects()
  {
    if (!detected_objects_.empty()) {
      std::string objects_list;
      for (const auto& obj : detected_objects_) {
        objects_list += obj + ", ";
      }
      // Remove the trailing comma and space
      if (!objects_list.empty()) {
        objects_list = objects_list.substr(0, objects_list.length() - 2);
      }
      // RCLCPP_INFO(this->get_logger(), "Detected objects: %s", objects_list.c_str());
    } else {
      // RCLCPP_INFO(this->get_logger(), "No objects detected");
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TrackObject::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Received goal request to track object: %s", goal->object_name.c_str());
    (void)uuid;
    // Check if the object is in the COCO-80 list (simplified for now)
    if (is_valid_object(goal->object_name)) {
      current_object_ = goal->object_name;
      // twist_msg->angular.z = SEARCH_SPEED;
      // twist_msg->linear.x = 0.0;
      is_tracking = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTrackObject> goal_handle)
  {
    // RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTrackObject> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleTrackObject> goal_handle)
  {
    // RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<TrackObject::Feedback>();
    auto action_result = std::make_shared<TrackObject::Result>();
    rclcpp::Rate loop_rate(10);  // 10 Hz feedback rate
    bool goal_completed = false;
    bool object_was_in_view = false;

    while (rclcpp::ok() && !goal_completed) {
      if (goal_handle->is_canceling()) {
        if (!goal_completed) {
          action_result->success = false;
          action_result->message = "Tracking Failed.";
          goal_handle->canceled(action_result);
          goal_completed = true;
          // RCLCPP_INFO(this->get_logger(), "Goal canceled");
        }
        return;
      }

      if (object_in_fov_) {
        if (!object_was_in_view) {
          object_was_in_view = true;
          feedback->message = "Object detected in view!";
          goal_handle->publish_feedback(feedback);
        }
        
        // Get the object's position and send appropriate feedback
        if (last_detection_ && last_depth_image_) {
          for (const auto & detection : last_detection_->detections) {
            for (const auto & detection_result : detection.results) {
              if (detection_result.hypothesis.class_id == current_object_) {
                // Calculate object position relative to center
                double center_x = detection.bbox.center.position.x;
                double center_y = detection.bbox.center.position.y;
                
                // Get depth value at object center
                int pixel_index = static_cast<int>(center_y) * last_depth_image_->width + static_cast<int>(center_x);
                if (pixel_index >= 0 && pixel_index < static_cast<int>(last_depth_image_->data.size() / 4)) {
                    // Calculate average depth over the entire ROI
                    double total_depth = 0.0;
                    int valid_pixels = 0;
                    
                    // Get bounding box coordinates
                    double bbox_x = detection.bbox.center.position.x - detection.bbox.size_x / 2;
                    double bbox_y = detection.bbox.center.position.y - detection.bbox.size_y / 2;
                    double bbox_width = detection.bbox.size_x;
                    double bbox_height = detection.bbox.size_y;
                    
                    // Constants for depth value filtering
                    const float MAX_VALID_DEPTH = 10.0f;  // 10 meters
                    const float MIN_VALID_DEPTH = 0.1f;   // 10 cm
                    
                    // Iterate through all pixels in the bounding box
                    for (int y = static_cast<int>(bbox_y); y < static_cast<int>(bbox_y + bbox_height); ++y) {
                        for (int x = static_cast<int>(bbox_x); x < static_cast<int>(bbox_x + bbox_width); ++x) {
                            // Check if pixel is within image bounds
                            if (x >= 0 && x < static_cast<int>(last_depth_image_->width) &&
                                y >= 0 && y < static_cast<int>(last_depth_image_->height)) {
                                
                                int current_pixel_index = y * last_depth_image_->width + x;
                                if (current_pixel_index >= 0 && current_pixel_index < static_cast<int>(last_depth_image_->data.size() / 4)) {
                                    // Get the 4 bytes for the float32 value
                                    const uint8_t* pixel_data = &last_depth_image_->data[current_pixel_index * 4];
                                    float depth_value;
                                    
                                    if (last_depth_image_->is_bigendian) {
                                        // Big endian: most significant byte first
                                        uint32_t raw_value = (static_cast<uint32_t>(pixel_data[0]) << 24) |
                                                           (static_cast<uint32_t>(pixel_data[1]) << 16) |
                                                           (static_cast<uint32_t>(pixel_data[2]) << 8) |
                                                           pixel_data[3];
                                        depth_value = *reinterpret_cast<const float*>(&raw_value);
                                    } else {
                                        // Little endian: least significant byte first
                                        uint32_t raw_value = (static_cast<uint32_t>(pixel_data[3]) << 24) |
                                                           (static_cast<uint32_t>(pixel_data[2]) << 16) |
                                                           (static_cast<uint32_t>(pixel_data[1]) << 8) |
                                                           pixel_data[0];
                                        depth_value = *reinterpret_cast<const float*>(&raw_value);
                                    }
                                    
                                    // Filter out invalid depth values
                                    if (depth_value > MIN_VALID_DEPTH && depth_value < MAX_VALID_DEPTH) {
                                        total_depth += depth_value;
                                        valid_pixels++;
                                    }
                                }
                            }
                        }
                    }
                    
                    if (valid_pixels > 0) {
                        double average_depth = total_depth / valid_pixels;
                        
                        // Update depth history
                        depth_history_.push_back(average_depth);
                        if (depth_history_.size() > HISTORY_SIZE) {
                            depth_history_.pop_front();
                        }
                        
                        // Calculate weighted average
                        double weighted_average = calculate_weighted_average();
                        
                        RCLCPP_INFO(this->get_logger(), 
                                  "Current depth: %.3f m, Weighted average (last %zu frames): %.3f m", 
                                  average_depth, depth_history_.size(), weighted_average);

                        // Check if object is close enough (less than 20cm)
                        if (weighted_average < 0.40) {  // 20cm in meters
                            is_tracking = false;
                            if (!goal_completed) {
                                action_result->success = true;
                                action_result->message = "Object is within 20cm!";
                                goal_handle->succeed(action_result);
                                goal_completed = true;
                                // RCLCPP_INFO(this->get_logger(), "Goal succeeded - object is within 20cm");
                            }
                            return;
                        }
                    }
                }
                break;
              }
            }
          }
        }
      } else if (object_was_in_view) {
        // Object was previously in view but now is out of FOV
        // Commented out the cancellation logic
        /*
        if (!goal_completed) {
            action_result->success = false;
            action_result->message = "Object is out of FOV.";
            goal_handle->canceled(action_result);
            goal_completed = true;
            // RCLCPP_INFO(this->get_logger(), "Goal canceled - object out of FOV");
            return;
        }
        */
        // Instead, just send feedback that object is out of view
        feedback->message = "Object is out of view, waiting for it to return...";
        goal_handle->publish_feedback(feedback);
      } else {
        // Object not in view yet, just wait
        feedback->message = "Waiting for object to come into view...";
      goal_handle->publish_feedback(feedback);
      }
      loop_rate.sleep();
    }
  }

  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Detection callback triggered with %zu detections", msg->detections.size());
    last_detection_ = msg;
    object_in_fov_ = false;
    detected_objects_.clear();

    // Check if the current object is in the detections and collect all detected objects
    for (const auto & detection : msg->detections) {
      for (const auto & result : detection.results) {
        std::string class_id = result.hypothesis.class_id;
        detected_objects_.insert(class_id);
        // RCLCPP_INFO(this->get_logger(), "Detected object: %s", class_id.c_str());
        
        if (class_id == current_object_) {
          object_in_fov_ = true;
          // RCLCPP_INFO(this->get_logger(), "Target object %s found in view!", current_object_.c_str());
        }
      }
    }
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Received depth image: %dx%d", msg->width, msg->height);
    last_depth_image_ = msg;
  }

  bool is_valid_object(const std::string & object_name)
  {
    // Check if the object is in the COCO-80 list
    // Simplified for now
    return true;
  }

  bool is_object_centered()
  {
    if (!last_detection_) return false;

    for (const auto & detection : last_detection_->detections) {
      for (const auto & result : detection.results) {
        if (result.hypothesis.class_id == current_object_) {
          // Check if object is centered within ±50px (more lenient than before)
          double center_x = detection.bbox.center.position.x;
          double center_y = detection.bbox.center.position.y;
          
          const double image_center_x = 320.0;  // Assuming 640x480 image
          const double image_center_y = 240.0;
          
          return (std::abs(center_x - image_center_x) < 50 && 
                  std::abs(center_y - image_center_y) < 50);
    }
      }
    }
    return false;
  }
};  // class MyActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<MyActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}