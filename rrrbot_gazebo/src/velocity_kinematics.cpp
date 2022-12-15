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

class VelocityKinematics : public rclcpp::Node
{
public:
  VelocityKinematics() : Node("vel_kinematics")
  {
    //Service for obtaining joint velocities from end effector velocity
    service_1 = this->create_service<tutorial_interfaces::srv::EndToJoint>("end_to_joint", std::bind(&VelocityKinematics::velref, this, _1));
    //Service for obtaining end effector velocity from joint velocities
    service_2 = this->create_service<tutorial_interfaces::srv::JointToEnd>("joint_to_end", std::bind(&VelocityKinematics::jref, this, _1));
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
                  request)
  {
    //Taking end effector velocity from request 
    evx_ref = request->vx_ref;
    evy_ref = request->vy_ref;
    evz_ref = request->vz_ref;
    ewx_ref = request->omx_ref;
    ewy_ref = request->omy_ref;
    ewz_ref = request->omz_ref;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nvx_ref: %f" " vy_ref: %f" " vz_ref: %f", request->vx_ref, request->vy_ref, request->vz_ref);
    //Subscribing to current joint states for getting joint angle values for computing Jacobian matrix 
    subscriber_1 = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&VelocityKinematics::topic_callback1, this, _1)); 
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response in service:\n [%f, %f, %f]", j1v_ref, j2v_ref, j3v_ref);
  }

  void topic_callback1(const sensor_msgs::msg::JointState &msg) const {
    //Extracting current position information 
    std::double_t q1 = msg.position[0];
    std::double_t q2 = msg.position[1];
    std::double_t q3 = msg.position[2];
    //Computing pseudo inverse of Jacobian matrix 
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
    //Eigen::MatrixXd pinvJinv = pinvJ.completeOrthogonalDecomposition().pseudoInverse();
    // for(int i=0;i<6;i++){
    //   for(int j=0;j<3;j++){
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pinvJinv matrix elem:\n %f", pinvJinv(i,j));
    //   }
    // }
    Eigen::MatrixXd vmat(6,1);
    vmat(0,0) = evx_ref;
    vmat(1,0) = evy_ref;
    vmat(2,0) = evz_ref;
    vmat(3,0) = ewx_ref;
    vmat(4,0) = ewy_ref;
    vmat(5,0) = ewz_ref;

    //Computing joint velocities from pseudo inverse of Jacobian matrix and end effector velocity 
    Eigen::MatrixXd jmat = pinvJ*vmat;
    j1v_ref = jmat(0,0);
    j2v_ref = jmat(1,0);
    j3v_ref = jmat(2,0);
    //Sending the obtained joint velocities as response 
    tutorial_interfaces::srv::EndToJoint::Response response;
    response.joint1vel_ref = j1v_ref;
    response.joint2vel_ref = j2v_ref;
    response.joint3vel_ref = j3v_ref;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response:\n [%f, %f, %f]", response.joint1vel_ref, response.joint2vel_ref, response.joint3vel_ref);
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_1;
  rclcpp::Service<tutorial_interfaces::srv::EndToJoint>::SharedPtr service_1;

private:
  mutable double j1_refv;
  mutable double j2_refv;
  mutable double j3_refv;
  mutable double v_refx;
  mutable double v_refy; 
  mutable double v_refz;

  void jref(const std::shared_ptr<tutorial_interfaces::srv::JointToEnd::Request>
                  request1)
  {
    //Taking joint velocities from request 
    j1_refv = request1->j1vel_ref;
    j2_refv = request1->j2vel_ref;
    j3_refv = request1->j3vel_ref;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nj1vel_ref: %f" " j2vel_ref: %f" " j3vel_ref: %f", request1->j1vel_ref, request1->j2vel_ref, request1->j3vel_ref);
    //Subscribing to current joint states for getting joint angle values for computing Jacobian matrix 
    subscriber_2 = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&VelocityKinematics::topic_callback2, this, _1)); 
  }

  void topic_callback2(const sensor_msgs::msg::JointState &msg2) const {
    //Extracting current position information
    std::double_t q1 = msg2.position[0];
    std::double_t q2 = msg2.position[1];
    std::double_t q3 = msg2.position[2];
    //Computing Jacobian matrix 
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

    Eigen::MatrixXd jvmat(3,1);
    jvmat(0,0) = j1_refv;
    jvmat(1,0) = j2_refv;
    jvmat(2,0) = j3_refv;

    //Computing end effector velocity from Jacobian matrix and joint velocities 
    Eigen::MatrixXd vemat = J*jvmat;
    v_refx = vemat(0,0);
    v_refy = vemat(1,0);
    v_refz = vemat(2,0);
    //Sending the obtained end effector joint velocities as response 
    tutorial_interfaces::srv::JointToEnd::Response response1;
    response1.vref_x = v_refx;
    response1.vref_y = v_refy;
    response1.vref_z = v_refz;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response:\n [%f, %f, %f]", response1.vref_x, response1.vref_y, response1.vref_z);
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_2;
  rclcpp::Service<tutorial_interfaces::srv::JointToEnd>::SharedPtr service_2;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityKinematics>());
  rclcpp::shutdown();
  return 0;
}