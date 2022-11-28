#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/theta_new_ref.hpp" 

#include <memory>

//Displays the reference angles when the service is called 
void send(std::shared_ptr<tutorial_interfaces::srv::ThetaNewRef::Request> request,std::shared_ptr<tutorial_interfaces::srv::ThetaNewRef::Response> response) 
{                           
  response->theta1_des = -0.3;
  response->theta2_des = 1.83;
  response->theta3_des = 0.8;        
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request for service, the desired angles are \ntheta1_des: %f" " theta2_des: %f" " theta3_des: %f",  
                response->theta1_des, response->theta2_des, response->theta3_des);                                        
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("theta_new_ref");
  
  rclcpp::Service<tutorial_interfaces::srv::ThetaNewRef>::SharedPtr service = node->create_service<tutorial_interfaces::srv::ThetaNewRef>("theta_new_ref",&send);

  rclcpp::spin(node);

  rclcpp::shutdown();
}
