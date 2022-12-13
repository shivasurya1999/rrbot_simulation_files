#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
using std::placeholders::_1;
using namespace std::chrono_literals;

// Creating the subscriber
// The below code just takes theta values into another message and publishes
// them for the main controller to use
class Processor : public rclcpp::Node {
public:
  Processor() : Node("pos_control") {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&Processor::topic_callback, this,
                  _1)); // subscribing to /joint_states
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "theta_actual", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

  void topic_callback(const sensor_msgs::msg::JointState &msg) const {
    std::double_t theta1 = msg.position[0];
    std::double_t theta2 = msg.position[1];
    std::double_t theta3 = msg.position[2];

    RCLCPP_INFO(this->get_logger(), "theta1= '%f',theta2='%f',theta3='%f'",
                theta1, theta2, theta3);

    std_msgs::msg::Float64MultiArray message;
    message.data.push_back(theta1);
    message.data.push_back(theta2);
    message.data.push_back(theta3);
    publisher_->publish(message); // publishing the actual joint
                                  // angles to the topic theta_actual
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Processor>());
  rclcpp::shutdown();
  return 0;
}