#include "firstrobot_webots/ObstacleAvoider.hpp"

#define MAX_RANGE 0.15  

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider") {
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  publisher_robot2_ = create_publisher<geometry_msgs::msg::Twist>("/robot2/cmd_vel", 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/left_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        return this->leftSensorCallback(msg);
      }
  );

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/right_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        return this->rightSensorCallback(msg);
      }
  );

  left_sensor_sub_robot2_ = create_subscription<sensor_msgs::msg::Range>(
      "/robot2/left_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        return this->leftSensorCallbackRobot2(msg);
      }
  );

  right_sensor_sub_robot2_ = create_subscription<sensor_msgs::msg::Range>(
      "/robot2/right_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        return this->rightSensorCallbackRobot2(msg);
      }
  );
}

void ObstacleAvoider::leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

void ObstacleAvoider::rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;

  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
  command_message->linear.x = 0.1;

  if ((left_sensor_value > 0.0 && left_sensor_value < 0.9 * MAX_RANGE) ||
      (right_sensor_value > 0.0 && right_sensor_value < 0.9 * MAX_RANGE)) {
    RCLCPP_INFO(this->get_logger(), "Obstáculo detectado (my_robot)! Esquerda: %.2f m, Direita: %.2f m", 
                left_sensor_value, right_sensor_value);
    command_message->linear.x = 2; 
  } else {
    RCLCPP_INFO(this->get_logger(), "Sem obstáculo (my_robot). Esquerda: %.2f m, Direita: %.2f m", 
                left_sensor_value, right_sensor_value);
  }

  // Limita os valores para evitar exceder maxVelocity
  if (command_message->linear.x > 0.1) command_message->linear.x = 0.1;

  publisher_->publish(std::move(command_message));
}

void ObstacleAvoider::leftSensorCallbackRobot2(const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value_robot2 = msg->range;
}

void ObstacleAvoider::rightSensorCallbackRobot2(const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value_robot2 = msg->range;

  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
  command_message->linear.x = 0.1;

  if ((left_sensor_value_robot2 > 0.0 && left_sensor_value_robot2 < 0.9 * MAX_RANGE) ||
      (right_sensor_value_robot2 > 0.0 && right_sensor_value_robot2 < 0.9 * MAX_RANGE)) {
    RCLCPP_INFO(this->get_logger(), "Obstáculo detectado (my_robot_2)! Esquerda: %.2f m, Direita: %.2f m", 
                left_sensor_value_robot2, right_sensor_value_robot2);
    command_message->linear.x = 2; 
  } else {
    RCLCPP_INFO(this->get_logger(), "Sem obstáculo (my_robot_2). Esquerda: %.2f m, Direita: %.2f m", 
                left_sensor_value_robot2, right_sensor_value_robot2);
  }

  // Limita os valores para evitar exceder maxVelocity
  if (command_message->linear.x > 0.1) command_message->linear.x = 0.1;

  publisher_robot2_->publish(std::move(command_message));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto avoider = std::make_shared<ObstacleAvoider>();
  rclcpp::spin(avoider);
  rclcpp::shutdown();
  return 0;
}