#include "firstrobot_webots/RobotSeeker.hpp"

#define MIN_FRONT_RANGE 0.12
#define MIN_SIDE_RANGE 0.06
#define MAX_SIDE_RANGE 0.15
#define OPTIMAL_DISTANCE 0.12 // Target distance from the wall (in meters)
#define DISTANCE_TOLERANCE 0.01  // Acceptable range around optimal distance
#define REVERSE_VEL -0.1
#define BASE_VEL 0.11
#define CHASE_VEL 0.09

RobotSeeker::RobotSeeker() : Node("robot_seeker") {

  std::string current_namespace = this->get_namespace();
  RCLCPP_INFO(this->get_logger(), "Identified current namespace on RobotSeeker :  %s", current_namespace.c_str());

  std::string cmd_vel_topic = current_namespace + "/cmd_vel";
  std::string front_sensor_topic = current_namespace + "/front_sensor";
  std::string right_sensor_topic = current_namespace + "/right_sensor";
  std::string camera_topic = current_namespace + "/camera/image_color";

  publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);

  front_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      front_sensor_topic, 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg){
        return this->frontSensorCallback(msg);
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

void RobotSeeker::frontSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  front_sensor_value = msg->range;
}

void RobotSeeker::rightSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;
}

void RobotSeeker::cameraCallback(
  const sensor_msgs::msg::Image::SharedPtr msg) {
  // Check if the encoding is bgra8
  if (msg->encoding != "bgra8") {
      RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
  }

  // Count green pixels on the left and right sides
  auto [green_pixels_left, green_pixels_right] = countGreenPixels(msg);

  // Create and populate the command message
  auto command_message = createCommandMessage(green_pixels_left, green_pixels_right);

  // Publish the command message
  publisher_->publish(std::move(command_message));
}

std::pair<int, int> RobotSeeker::countGreenPixels(const sensor_msgs::msg::Image::SharedPtr msg) {
  int green_pixels_left = 0;
  int green_pixels_right = 0;
  size_t image_size = msg->data.size();
  int width = msg->width;

  // Iterate through the image data
  for (size_t i = 0; i < image_size; i += 4) {
      uint8_t blue = msg->data[i];
      uint8_t green = msg->data[i + 1];
      uint8_t red = msg->data[i + 2];

      // Calculate saturation
      uint8_t saturation = std::max({red, green, blue}) - std::min({red, green, blue});

      // Check if green is the predominant color and saturation is high
      if (green > red && green > blue && saturation > 60) {
          size_t pixel_index = i / 4;
          int pixel_x = pixel_index % width;

          if (pixel_x < width / 2) {
              green_pixels_left++;
          } else {
              green_pixels_right++;
          }
      }
  }

  return {green_pixels_left, green_pixels_right};
}

std::unique_ptr<geometry_msgs::msg::Twist> RobotSeeker::createCommandMessage(
  int green_pixels_left, int green_pixels_right) {
  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

  command_message->linear.x = BASE_VEL;
  command_message->angular.z = 0.0;

  // If too close to obstacle or wall, reverse
  if (right_sensor_value < MIN_SIDE_RANGE * 1.1 || front_sensor_value < MIN_FRONT_RANGE) {
      command_message->linear.x = REVERSE_VEL;
  }
  // If robot is closer than wall
  else if (front_sensor_value <= right_sensor_value || front_sensor_value < 0.1) {
      if (green_pixels_left > 0 || green_pixels_right > 0) {
          command_message->linear.x = CHASE_VEL;
          if (green_pixels_left > green_pixels_right) {
              command_message->angular.z = 1.0;
          } else if (green_pixels_right > green_pixels_left) {
              command_message->angular.z = -1.0;
          }
      } else if (right_sensor_value < MAX_SIDE_RANGE * 0.95) {
          command_message->linear.x = 0.0;
          command_message->angular.z = 0.0;
      } else {
          command_message->linear.x = 0.0;
          command_message->angular.z = -1.0;
      }
  }
  // Wall is closer than robot
  else {
      if (right_sensor_value > MAX_SIDE_RANGE * 0.95) {
          command_message->linear.x = 0.0;
          command_message->angular.z = -1.0;
      } else if (right_sensor_value > OPTIMAL_DISTANCE + DISTANCE_TOLERANCE) {
          command_message->angular.z = -1.0;
      } else if (right_sensor_value < OPTIMAL_DISTANCE - DISTANCE_TOLERANCE) {
          command_message->angular.z = 1.0;
      }
  }

  return command_message;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto seeker = std::make_shared<RobotSeeker>();
  rclcpp::spin(seeker);
  rclcpp::shutdown();
  return 0;
}