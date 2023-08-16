//
// Created by jimmy on 23-8-8.
//

#include <rclcpp/rclcpp.hpp>
#include "OdometryServer.hpp"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto ros_opt = rclcpp::NodeOptions();

    auto node = std::make_shared<kiss_icp_ros::OdometryServer>(ros_opt);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
