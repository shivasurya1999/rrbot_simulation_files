#include <chrono>
#include <functional>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include "sensor_msgs/msg/joint_state.hpp"
#include "tutorial_interfaces/srv/end_to_joint.hpp"
#include "tutorial_interfaces/srv/joint_to_end.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iterator>
#include <optional>
#include <ratio>
#include <thread>
#include <time.h>
#include <utility>

using std::placeholders::_1;
using namespace std::chrono_literals;

class VelocityServer : public std::shared_ptr<rclcpp::Node>
{
public:
  VelocityServer() : std::shared_ptr<rclcpp::Node("velocity_server")>
  {
    rclcpp::Service<tutorial_interfaces::srv::EndToJoint>::SharedPtr service = VelocityServer->create_service<tutorial_interfaces::srv::EndToJoint>("end_to_joint", &velref);
    //service_1 = this->create_service<tutorial_interfaces::srv::EndToJoint>("end_to_joint", std::bind(&VelocityKinematics::velref, this, _1));
    //service_2 = this->create_service<tutorial_interfaces::srv::JointToEnd>("joint_to_end", std::bind(&VelocityKinematics::jref, this, _1));
  }

private:
  mutable double evx_ref;
  mutable double evy_ref;
  mutable double evz_ref;
  mutable double ewx_ref;
  mutable double ewy_ref;
  mutable double ewz_ref;
  mutable double j1v_ref;
  mutable double j2v_ref; 
  mutable double j3v_ref;

  void velref(const std::shared_ptr<tutorial_interfaces::srv::EndToJoint::Request>
                  request, std::shared_ptr<tutorial_interfaces::srv::EndToJoint::Response> response)
  {
    evx_ref = request->vx_ref;
    evy_ref = request->vy_ref;
    evz_ref = request->vz_ref;
    ewx_ref = request->omx_ref;
    ewy_ref = request->omy_ref;
    ewz_ref = request->omz_ref;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nvx_ref: %f" " vy_ref: %f" " vz_ref: %f", request->vx_ref, request->vy_ref, request->vz_ref);
    subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&VelocityServer::topic_callback, this, _1)); 
    response->joint1vel_ref = j1v_ref;
    response->joint2vel_ref = j2v_ref;
    response->joint3vel_ref = j3v_ref;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response:\n [%f, %f, %f]", response->joint1vel_ref, response->joint2vel_ref, response->joint3vel_ref);
  }

  void topic_callback(const sensor_msgs::msg::JointState &msg) const {
    std::double_t q1 = msg.position[0];
    std::double_t q2 = msg.position[1];
    std::double_t q3 = msg.position[2];
    double sig1 = sin(q1)*sin(q2);
    double sig2 = cos(q2)*sin(q1);
    double sig3 = cos(q1)*sin(q2);
    double sig4 = cos(q1)*cos(q2);
    double sq1 = sin(q1);
    double cq1 = cos(q1);
    Eigen::MatrixXd J(6,3);
    J(0,0) = -sq1-sig3-sig2;
    J(0,1) = -sig3-sig2;
    J(0,2) = 0;
    J(1,0) = cq1+sig4-sig1;
    J(1,1) = sig4-sig1;
    J(1,2) = 0;
    J(2,0) = 0;
    J(2,1) = 0;
    J(2,2) = -1;
    J(3,0) = 0;
    J(3,1) = 0;
    J(3,2) = 0;
    J(4,0) = 0;
    J(4,1) = 0;
    J(4,2) = 0;
    J(5,0) = 1;
    J(5,1) = 1;
    J(5,2) = 0;
    Eigen::MatrixXd pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd vmat(6,1);
    vmat(0,0) = evx_ref;
    vmat(1,0) = evy_ref;
    vmat(2,0) = evz_ref;
    vmat(3,0) = ewx_ref;
    vmat(4,0) = ewy_ref;
    vmat(5,0) = ewz_ref;

    Eigen::MatrixXd jmat = pinvJ*vmat;
    j1v_ref = jmat(0,0);
    j2v_ref = jmat(1,0);
    j3v_ref = jmat(2,0);

  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber;
  rclcpp::Service<tutorial_interfaces::srv::EndToJoint>::SharedPtr service;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityServer>());
  rclcpp::shutdown();
  return 0;
}