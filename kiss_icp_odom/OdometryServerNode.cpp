//
// Created by jimmy on 23-8-8.
//

#include "OdometryServer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cartographer_ros/ros_log_sink.h>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto ros_opt = rclcpp::NodeOptions();
    rclcpp::executors::MultiThreadedExecutor e;
    cartographer_ros::ScopedRosLogSink ros_log_sink;
    auto node = std::make_shared<kiss_icp_ros::OdometryServer>(ros_opt);
    e.add_node(node);
    e.spin();
    rclcpp::shutdown();
    return 0;
}
