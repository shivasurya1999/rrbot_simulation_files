#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp" 
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iostream>
#include <time.h>
#include <cstdlib>
#include <ratio>
#include <thread>
#include<optional>
#include <utility>
#include <vector>
#include <iterator>
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std::chrono;


class NewProcessor : public rclcpp::Node
{
  public:
    NewProcessor()
    : Node("main_control")
    {
      subscription_1 = this->create_subscription<std_msgs::msg::Float64MultiArray>("theta_des_and_actual", 10,std::bind(&NewProcessor::topic1_callback, this, _1));
      publisher_1 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_effort_controller/commands", 10);
    }

  private:
    //setting the old error to be the difference in desired and actual angles 
    // std::optional<double> e1_old = {0};
    // std::optional<double> e2_old = {0};
    // std::optional<double> e3_old = {0};
    mutable double e1_old;
    mutable double e2_old;
    mutable double e3_old;

    void topic1_callback(const std_msgs::msg::Float64MultiArray & msg) const
    {
        std::double_t theta1_des = msg.data[0];
        std::double_t theta2_des = msg.data[1];
        std::double_t theta3_des = msg.data[2];
        std::double_t theta1 = msg.data[3];
        std::double_t theta2 = msg.data[4];
        std::double_t theta3 = msg.data[5];
        RCLCPP_INFO(this->get_logger(), "theta1_des= '%f',theta2_des='%f',theta3_des='%f'",theta1_des,theta2_des,theta3_des);
        RCLCPP_INFO(this->get_logger(), "theta1= '%f',theta2='%f',theta3='%f'",theta1,theta2,theta3);
      
        //initial joint efforts are set to be zero 
        double joint1_effort = 0;
        double joint2_effort = 0;
        double joint3_effort = 0;
        //steady state error is set to be 0.05 
        double epsilon = 0.05;
        //initializing error variables 
        double e1,e2,e3,e1_dot,e2_dot,e3_dot;
        //the sampling time is 100 milliseconds 
        double sampling_time = 0.1;
        //Setting the proportional and derivative gains 
        double Kp1 = 0.04;
        double Kd1 = 0.03;
        double Kp2 = 0.04;
        double Kd2 = 0.03;
        double Kp3 = 125;
        double Kd3 = 60;

        //creating message for publisher_ to publish efforts 
        //auto message = std_msgs::msg::Float64MultiArray();
        std_msgs::msg::Float64MultiArray message;
        //Calculating joint efforts using PD control parameters set above and running the loop till all have reached the necessary steady state error 
        if((std::abs(theta1_des-theta1)>epsilon)){
            e1 = theta1_des - theta1;
            if(std::abs(theta1)<0.1){
              joint1_effort = Kp1*e1;
              e1_old = e1;
            }
            else{
              e1_dot = (e1-e1_old)/sampling_time;
              joint1_effort = Kp1*e1 + Kd1*e1_dot; //effort for joint1
              e1_old = e1;
            }
        }
        if((std::abs(theta2_des-theta2)>epsilon)){
            e2 = theta2_des - theta2;
            if(std::abs(theta2)<0.15){
              joint2_effort = Kp2*e2;
              e2_old = e2;
            }
            else{
              e2_dot = (e2-e2_old)/sampling_time;
              joint2_effort = Kp2*e2 + Kd2*e2_dot; //effort for joint2
              e2_old = e2;
            }
        }
        if((std::abs(theta3_des-theta3)>epsilon)){
            e3 = theta3_des - theta3;
            if(std::abs(theta3)<0.01){
              joint3_effort = Kp3*e3;
              e3_old = e3;
            }
            else{
              e3_dot = (e3-e3_old)/sampling_time;
              joint3_effort = Kp3*e3 + Kd3*e3_dot; //effort for joint3
              e3_old = e3;
            }
        }   
        message.data.push_back(joint1_effort);
        message.data.push_back(joint2_effort);
        message.data.push_back(joint3_effort);
        RCLCPP_INFO(this->get_logger(), "joint1e= '%f',joint2e='%f',joint3e='%f'",joint1_effort,joint2_effort,joint3_effort);
        publisher_1->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_1;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Rate loop_rate(10); //10 Hz frequency 
  //rclcpp::spin(std::make_shared<NewProcessor>());
  while(rclcpp::ok){
    rclcpp::spin_some(std::make_shared<NewProcessor>());
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}