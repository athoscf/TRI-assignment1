#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/image.hpp"

class RobotSeeker: public rclcpp::Node {
public:
  explicit RobotSeeker();

private:
  void frontSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr front_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;

  double front_sensor_value{0.0};
  double right_sensor_value{0.0};

  std::pair<int, int> countGreenPixels(const sensor_msgs::msg::Image::SharedPtr msg);
  std::unique_ptr<geometry_msgs::msg::Twist> createCommandMessage(int green_pixels_left, int green_pixels_right);

};