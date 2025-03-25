#include "firstrobot_webots/RobotSeeker.hpp"

#define MAX_RANGE 0.15
#define MIN_RANGE 0.06
#define OPTIMAL_DISTANCE 0.12 // Target distance from the wall (in meters)
#define DISTANCE_TOLERANCE 0.01  // Acceptable range around optimal distance
#define REVERSE_VEL -0.05
#define CHASE_VEL 0.12

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
}

void RobotSeeker::cameraCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
      // Check if the encoding is bgra8
      if (msg->encoding != "bgra8") {
        RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
        return;
      }
  
      int green_pixels_right = 0;
      int green_pixels_left = 0;
      size_t image_size = msg->data.size();
      int width = msg->width; // Image width in pixels
      int height = msg->height; // Image height in pixels
  
      // Iterate through the image data
      for (size_t i = 0; i < image_size; i += 4) {
        // Extract color channels
        uint8_t blue = msg->data[i];
        uint8_t green = msg->data[i + 1];
        uint8_t red = msg->data[i + 2];
  
        // Calculate saturation (max - min of RGB values)
        uint8_t saturation = std::max({red, green, blue}) - std::min({red, green, blue});
  
        // Check if green is the predominant color and saturation is high
        if (green > red && green > blue && saturation > 60) { // Adjust saturation threshold as needed
          // Determine if the pixel is on the left or right side of the image
          size_t pixel_index = i / 4; // Pixel index in the image
          int pixel_x = pixel_index % width; // X-coordinate of the pixel
  
          if (pixel_x < width / 2) {
            green_pixels_left++;
          } else {
            green_pixels_right++;
          }
        }
      }

      auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

      // Constant forward velocity
      command_message->linear.x = 0.1;  
      command_message->angular.z = 0.0;  // Default to no rotation

      if (right_sensor_value < MIN_RANGE * 1.1){
        command_message->linear.x = REVERSE_VEL;
      }
      else if (green_pixels_left > 0 || green_pixels_right > 0) {
        command_message->linear.x = CHASE_VEL;
        if (green_pixels_left > green_pixels_right){
            command_message->angular.z = 1.0;
        }
        else if (green_pixels_right > green_pixels_left){
            command_message->angular.z = -1.0;
        }
      }
      else if (right_sensor_value > MAX_RANGE * 0.95){
        command_message->linear.x = 0.0;
        command_message->angular.z = -1.0;
      }
      else if (right_sensor_value > OPTIMAL_DISTANCE + DISTANCE_TOLERANCE){
        command_message->angular.z = -1.0;
      }
      else if (right_sensor_value < OPTIMAL_DISTANCE - DISTANCE_TOLERANCE){
        command_message->angular.z = 1.0;
      }

    publisher_->publish(std::move(command_message));
  
    // Log the results
    RCLCPP_INFO(this->get_logger(), "Green pixels - Left: %d, Right: %d", green_pixels_left, green_pixels_right);
  }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto seeker = std::make_shared<RobotSeeker>();
  rclcpp::spin(seeker);
  rclcpp::shutdown();
  return 0;
}