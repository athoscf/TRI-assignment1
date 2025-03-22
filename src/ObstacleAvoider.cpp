#include "firstrobot_webots/ObstacleAvoider.hpp"

#define MAX_RANGE 0.15

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider") {

  std::string current_namespace = this->get_namespace();
  RCLCPP_INFO(this->get_logger(), "Identified current namespace on ObstacleAvoider :  %s", current_namespace.c_str());

  std::string cmd_vel_topic = current_namespace + "/cmd_vel";
  std::string left_sensor_topic = current_namespace + "/left_sensor";
  std::string right_sensor_topic = current_namespace + "/right_sensor";

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
}

void ObstacleAvoider::leftSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

void ObstacleAvoider::rightSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;

  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

  command_message->linear.x = 0.1;

  if (left_sensor_value < 0.9 * MAX_RANGE ||
      right_sensor_value < 0.9 * MAX_RANGE) {
    command_message->angular.z = -2.0;
  }

  publisher_->publish(std::move(command_message));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto avoider = std::make_shared<ObstacleAvoider>();
  rclcpp::spin(avoider);
  rclcpp::shutdown();
  return 0;
}