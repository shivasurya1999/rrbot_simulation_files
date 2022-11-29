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
      //timer_ = this->create_wall_timer(100ms, std::bind(&NewProcessor::topic1_callback, this));
    }

  private:
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
        //auto message = std_msgs::msg::Float64MultiArray();
        //setting the old error to be the difference in desired and actual angles 
        double e1_old = theta1_des-theta1;
        double e2_old = theta2_des-theta2;
        double e3_old = theta3_des-theta3;
        //initial joint efforts are set to be zero 
        double joint1_effort = 0;
        double joint2_effort = 0;
        double joint3_effort = 0;
        //steady state error is set to be 0.05 
        double epsilon = 0.2;
        //initializing error variables 
        double e1,e1_dot,e2,e2_dot,e3,e3_dot;
        //the sampling time is 50 milliseconds 
        double sampling_time = 0.05;
        //Setting the proportional and derivative gains 
        double Kp1 = 1;
        double Kd1 = 16;
        double Kp2 = 1;
        double Kd2 = 16;
        double Kp3 = 1;
        double Kd3 = 16;
        //creating message for publisher_ to publish efforts 
        std_msgs::msg::Float64MultiArray message;
        //Calculating joint efforts using PD control parameters set above and running the loop till all have reached the necessary steady state error 
        auto start = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> moment = 0.1s;
        auto end = std::chrono::high_resolution_clock::now();
        while((std::abs(theta1_des-theta1)>epsilon)||(std::abs(theta2_des-theta2)>epsilon)||(std::abs(theta3_des-theta3)>epsilon)){
            theta1 = msg.data[3];
            theta2 = msg.data[4];
            theta3 = msg.data[5];
            RCLCPP_INFO(this->get_logger(), "theta1= '%f',theta2='%f',theta3='%f'",theta1,theta2,theta3);
            if(moment.count()<0.07){
              end = std::chrono::high_resolution_clock::now();
              moment = end - start;
              RCLCPP_INFO(this->get_logger(), "joint1es= '%f',joint2es='%f',joint3es='%f'",joint1_effort,joint2_effort,joint3_effort);
              RCLCPP_INFO(this->get_logger(), "moment.count()= '%f'",moment.count());
              // publisher_1->publish(message);
              continue;
            }
            else{
              start = std::chrono::high_resolution_clock::now();
              if((std::abs(theta1_des-theta1)>epsilon)){
                  e1 = theta1_des - theta1;
                  e1_dot = (e1-e1_old)/sampling_time;
                  joint1_effort = Kp1*e1 + Kd1*e1_dot; //effort for joint1
                  e1_old = e1;
              }
              if((std::abs(theta2_des-theta2)>epsilon)){
                  e2 = theta2_des - theta2;
                  e2_dot = (e2-e2_old)/sampling_time;
                  joint2_effort = Kp2*e2 + Kd2*e2_dot; //effort for joint2
                  e2_old = e2;
              }
              if((std::abs(theta3_des-theta3)>epsilon)){
                  e3 = theta3_des - theta3;
                  e3_dot = (e3-e3_old)/sampling_time;
                  joint3_effort = Kp3*e3 + Kd3*e3_dot; //effort for joint3
                  e3_old = e3;
              }   
              message.data.push_back(joint1_effort);
              message.data.push_back(joint2_effort);
              message.data.push_back(joint3_effort);
              //RCLCPP_INFO(this->get_logger(), "joint1e= '%f',joint2e='%f',joint3e='%f'",joint1_effort,joint2_effort,joint3_effort);
              publisher_1->publish(message);
              auto end = std::chrono::high_resolution_clock::now();
              moment = end - start;
            }
            // if((std::abs(theta1_des-theta1)>epsilon)){
            //     e1 = theta1_des - theta1;
            //     e1_dot = (e1-e1_old)/sampling_time;
            //     joint1_effort = Kp1*e1 + Kd1*e1_dot; //effort for joint1
            //     e1_old = e1;
            // }
            // if((std::abs(theta2_des-theta2)>epsilon)){
            //     e2 = theta2_des - theta2;
            //     e2_dot = (e2-e2_old)/sampling_time;
            //     joint2_effort = Kp1*e2 + Kd1*e2_dot; //effort for joint2
            //     e2_old = e2;
            // }
            // if((std::abs(theta3_des-theta3)>epsilon)){
            //     e3 = theta3_des - theta3;
            //     e3_dot = (e3-e3_old)/sampling_time;
            //     joint3_effort = Kp1*e3 + Kd1*e3_dot; //effort for joint3
            //     e3_old = e3;
            // }   
            // message.data.push_back(joint1_effort);
            // message.data.push_back(joint2_effort);
            // message.data.push_back(joint3_effort);
            // RCLCPP_INFO(this->get_logger(), "joint1e= '%f',joint2e='%f',joint3e='%f'",joint1_effort,joint2_effort,joint3_effort);
            // publisher_1->publish(message);
            // auto end = high_resolution_clock::now();
            // duration = duration_cast<milliseconds>(end - start);
        }
    }
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_1;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::Rate loop_rate(10); //10 Hz frequency 
  rclcpp::spin(std::make_shared<NewProcessor>());
  // while(rclcpp::ok){
  //   rclcpp::spin_some(std::make_shared<NewProcessor>());
  //   loop_rate.sleep();
  // }
  rclcpp::shutdown();
  return 0;
}