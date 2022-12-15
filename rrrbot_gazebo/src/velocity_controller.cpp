#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tutorial_interfaces/srv/joint_to_end.hpp"
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <ratio>
#include <thread>
#include <time.h>
#include <utility>
#include <vector>
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std::chrono;

class NewVelProcessor : public rclcpp::Node {
public:
  NewVelProcessor() : Node("velocity_control") {
    // create a service to take desired joint values
    service_ = this->create_service<tutorial_interfaces::srv::JointToEnd>(
        "joint_to_end", std::bind(&NewVelProcessor::jointvelref, this, _1));
  }

private:
  // initializing the old error and desired joint values for modification inside
  // the callback function
  mutable double e1_old;
  mutable double e2_old;
  mutable double e3_old;
  mutable double jv1_des;
  mutable double jv2_des;
  mutable double jv3_des;

  void jointvelref(
      const std::shared_ptr<tutorial_interfaces::srv::JointToEnd::Request> request) {
    // taking desired angle values from the client request from the terminal
    jv1_des = request->j1vel_ref;
    jv2_des = request->j2vel_ref;
    jv3_des = request->j3vel_ref;
    // subscribing to actual joint velocities
    subscription_2 = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&NewVelProcessor::topic2_callback, this, _1)); 
    publisher_3 = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_effort_controller/commands", 10); // publishing joint efforts
    publisher_4 = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/joint_vel_des", 10); // publishing desired joint values for plotting
  }

  void topic2_callback(const sensor_msgs::msg::JointState &msg) const {
    // extracting the actual joint values
    std::double_t jv1 = msg.velocity[0];
    std::double_t jv2 = msg.velocity[1];
    std::double_t jv3 = msg.velocity[2];
    RCLCPP_INFO(this->get_logger(),
                "\njv1_des= '%f',jv2_des='%f',jv3_des='%f'\n",
                jv1_des, jv2_des, jv3_des);
    RCLCPP_INFO(this->get_logger(), "\njv1= '%f',jv2='%f',jv3='%f'\n",
                jv1, jv2, jv3);

    // initial joint efforts are set to be zero
    double joint1_effort = 0;
    double joint2_effort = 0;
    double joint3_effort = 0;
    // steady state error is set to be 0.01
    double epsilon = 0.01;
    // initializing error variables
    double e1, e2, e3, e1_dot, e2_dot, e3_dot;
    // the sampling time is 100 milliseconds
    double sampling_time = 0.1;
    // Setting the proportional and derivative gains
    // Below are the tuned values
    double Kp1 = 3; // joint 1 proportional gain
    double Kd1 = 1; // joint 1 derivative gain
    double Kp2 = 3; // joint 2 proportional gain
    double Kd2 = 1; // joint 2 derivative gain
    double Kp3 = 100; // joint 3 proportional gain
    double Kd3 = 90;    // joint 3 derivative gain

    // creating message for publisher_1 to publish efforts
    std_msgs::msg::Float64MultiArray message;
    // creating message for publisher_2 to publish the desired theta values
    std_msgs::msg::Float64MultiArray message1;

    // Calculating joint efforts using PD control parameters set above and
    // calculating joint efforts till all joint values have reached the
    // necessary steady state

    // joint 1
    if ((std::abs(jv1_des - jv1) > epsilon)) {
      e1 = jv1_des - jv1; // calculating error
      if (std::abs(jv1) <
          0.03) { // During the very initial state, as there is no old error,
                  // using proportional controller only
        joint1_effort = Kp1 * e1; // effort for joint1
        e1_old = e1;              // updating old error
      } else {
        e1_dot = (e1 - e1_old) /
                 sampling_time; // calculating the rate of change of error
        joint1_effort =
            Kp1 * e1 +
            Kd1 * e1_dot; // effort for joint1 generated using PD controller
        e1_old = e1;      // updating old error
      }
    }

    // joint 2
    if ((std::abs(jv2_des - jv2) > epsilon)) {
      e2 = jv2_des - jv2;
      if (std::abs(jv2) < 0.15) {
        joint2_effort = Kp2 * e2;
        e2_old = e2;
      } else {
        e2_dot = (e2 - e2_old) / sampling_time;
        joint2_effort = Kp2 * e2 + Kd2 * e2_dot; // effort for joint2
        e2_old = e2;
      }
    }

    // joint 3
    if ((std::abs(jv3_des - jv3) > epsilon)) {
      e3 = jv3_des - jv3;
      if (std::abs(jv3) < 0.01) {
        joint3_effort = Kp3 * e3;
        e3_old = e3;
      } else {
        e3_dot = (e3 - e3_old) / sampling_time;
        joint3_effort = Kp3 * e3 + Kd3 * e3_dot; // effort for joint3
        e3_old = e3;
      }
    }

    // storing the joint efforts in a message
    message.data.push_back(joint1_effort);
    message.data.push_back(joint2_effort);
    message.data.push_back(joint3_effort);
    // storing desired joint values in another message
    message1.data.push_back(jv1_des);
    message1.data.push_back(jv2_des);
    message1.data.push_back(jv3_des);
    RCLCPP_INFO(
        this->get_logger(),
        "\njoint1 effort = '%f',joint2 effort ='%f',joint3 effort ='%f'\n",
        joint1_effort, joint2_effort, joint3_effort);
    // publishing joint efforts
    publisher_3->publish(message);
    // publishing desired angles taken from the service
    publisher_4->publish(message1);
  }
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_3;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_4;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      subscription_2;
  rclcpp::Service<tutorial_interfaces::srv::JointToEnd>::SharedPtr
      service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NewVelProcessor>());
  rclcpp::shutdown();
  return 0;
}