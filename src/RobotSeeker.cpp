#include "firstrobot_webots/RobotSeeker.hpp"

#define MAX_RANGE 0.15
#define MIN_RANGE 0.06
#define OPTIMAL_DISTANCE 0.12 // Target distance from the wall (in meters)
#define DISTANCE_TOLERANCE 0.01  // Acceptable range around optimal distance
#define REVERSE_VEL -0.05

RobotSeeker::RobotSeeker() : Node("robot_seeker") {

  std::string current_namespace = this->get_namespace();
  RCLCPP_INFO(this->get_logger(), "Identified current namespace on ObstacleAvoider :  %s", current_namespace.c_str());

  std::string cmd_vel_topic = current_namespace + "/cmd_vel";
  std::string left_sensor_topic = current_namespace + "/left_sensor";
  std::string right_sensor_topic = current_namespace + "/right_sensor";
  std::string camera_topic = current_namespace + "/camera/image_color";

  publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      left_sensor_topic, 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg){
        return this->leftSensorCallback(msg);
      }
  );

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      right_sensor_topic, 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg){
        return this->rightSensorCallback(msg);
      }
  );

  camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic, 1,
      [this](const sensor_msgs::msg::Image::SharedPtr msg){
        return this->cameraCallback(msg);
      }
  );

}

void RobotSeeker::leftSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

void RobotSeeker::rightSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;

  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

  // Constant forward velocity
  command_message->linear.x = 0.1;  
  command_message->angular.z = 0.0;  // Default to no rotation
  

  if (right_sensor_value < MAX_RANGE * 0.95){
    if (right_sensor_value < MIN_RANGE * 1.1){
      command_message->linear.x = REVERSE_VEL;
    } 
    else{
      // Wall following logic for right side
      if (right_sensor_value > OPTIMAL_DISTANCE + DISTANCE_TOLERANCE) {
        // Too far from wall, turn right
        command_message->angular.z = -1.0;
      } 
      else if (right_sensor_value < OPTIMAL_DISTANCE - DISTANCE_TOLERANCE) {
        // Too close to wall, turn left
        command_message->angular.z = 1.0;
      }
    } 
  } 
  else { //right sensor does not detect any object
    // keep turning right in place
    command_message->linear.x = 0.0;  
    command_message->angular.z = -1.0; 
  }

  publisher_->publish(std::move(command_message));
}

void RobotSeeker::cameraCallback(
  const sensor_msgs::msg::Image::SharedPtr msg) {
    // Check if the encoding is bgr8
    if (msg->encoding != "bgra8") {
      RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    // Iterate through the image data
    for (size_t i = 0; i < msg->data.size(); i += 3) {
      // Extract the blue channel (first byte in bgr8 encoding)
      blue_channel.push_back(msg->data[i]);
    }

    // Example: Log the size of the blue channel vector
    RCLCPP_INFO(this->get_logger(), "Extracted %zu blue channel values", blue_channel.size());
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto seeker = std::make_shared<RobotSeeker>();
  rclcpp::spin(seeker);
  rclcpp::shutdown();
  return 0;
}