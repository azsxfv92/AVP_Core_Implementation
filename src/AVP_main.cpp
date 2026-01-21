#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv){
    // ROS2 init
    rclcpp::init(argc, argv);
    // create node
    auto node = rclcpp::Node::make_shared("avp_main_node");
    // print log
    RCLCPP_INFO(node->get_logger(), "=== AVP Main Node has started ===");
    // finish ROS2
    rclcpp::shutdown();
    return 0;
}