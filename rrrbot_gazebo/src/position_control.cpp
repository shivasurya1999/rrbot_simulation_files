#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tutorial_interfaces/srv/theta_new_ref.hpp"  
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cstdlib>
using std::placeholders::_1;
using namespace std::chrono_literals;


//Creating the subscriber 
class Processor : public rclcpp::Node
{
  public:
  Processor(): Node("pos_control")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&Processor::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("theta_actual", 10);
  }


  private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    
    void topic_callback(const sensor_msgs::msg::JointState & msg) const
    {
        std::double_t theta1 = msg.position[0];
        std::double_t theta2 = msg.position[1];
        std::double_t theta3 = msg.position[2];
        
        //setting the request variables as the desired angles 
        // double theta1_des = result.get()->theta1_des; 
        // double theta2_des = result.get()->theta2_des; 
        // double theta3_des = result.get()->theta3_des;

        // double theta1_des = -0.8; 
        // double theta2_des = 2.05; 
        // double theta3_des = -0.3;

        // RCLCPP_INFO(this->get_logger(), "theta1_des= '%f',theta2_des='%f',theta3_des='%f'",theta1_des,theta2_des,theta3_des);
        RCLCPP_INFO(this->get_logger(), "theta1= '%f',theta2='%f',theta3='%f'",theta1,theta2,theta3);

        std_msgs::msg::Float64MultiArray message;
        // message.data.push_back(theta1_des); 
        // message.data.push_back(theta2_des);
        // message.data.push_back(theta3_des);
        message.data.push_back(theta1);
        message.data.push_back(theta2);
        message.data.push_back(theta3);
        publisher_->publish(message); //publishing the desired and actual joint angles to the topic theta_des_and_actual
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Processor>());
  // std::shared_ptr<rclcpp::Node> node1 = rclcpp::Node::make_shared("theta_new_ref_client");
  // rclcpp::Client<tutorial_interfaces::srv::ThetaNewRef>::SharedPtr client = node1->create_client<tutorial_interfaces::srv::ThetaNewRef>("theta_new_ref_client");
  // auto request = std::make_shared<tutorial_interfaces::srv::ThetaNewRef::Request>();
  // auto result = client->async_send_request();
  rclcpp::shutdown();
  return 0;
}