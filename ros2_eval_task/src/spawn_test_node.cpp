/**
 * @file spawn_test_node.cpp
 * @brief Simple test node to spawn and delete a Gazebo model using GazeboUtilsClient.
 * 
 * This node demonstrates how to use the GazeboUtilsClient to:
 * 1. Load a model SDF file from the package.
 * 2. Spawn the model at a specified pose in Gazebo.
 * 3. Wait for a short duration.
 * 4. Delete the model from Gazebo.
 * 
 * Author: Basel Amr Barakat
 * Email: baselamr52@gmail.com
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <chrono>

#include "ros2_eval_task/gazebo_utils_client.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a ROS2 node
    auto node = rclcpp::Node::make_shared("spawn_test_node");

    // Create Gazebo utility client
    auto utils = std::make_shared<ros2_eval_task::GazeboUtilsClient>(node);

    // Load the SDF model file from the package
    std::string pkg_share = ament_index_cpp::get_package_share_directory("ros2_eval_task");
    std::string sdf_path = pkg_share + "/models/battery_9v_leader/model.sdf";

    std::ifstream ifs(sdf_path);
    if (!ifs) {
        RCLCPP_ERROR(node->get_logger(), "Could not open model file: %s", sdf_path.c_str());
        rclcpp::shutdown();
        return 1;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string xml = buffer.str();

    // Define the pose for the model
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 1.1;  // Spawn height
    pose.orientation.w = 1.0;  // No rotation

    std::string model_name = "test_battery";

    // Spawn the model in Gazebo
    RCLCPP_INFO(node->get_logger(), "Spawning model: %s", model_name.c_str());
    if (!utils->spawn_model(model_name, xml, pose)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to spawn model: %s", model_name.c_str());
        rclcpp::shutdown();
        return 1;
    }

    // Wait for a few seconds to observe the model
    rclcpp::sleep_for(5s);

    // Delete the model from Gazebo
    RCLCPP_INFO(node->get_logger(), "Deleting model: %s", model_name.c_str());
    if (!utils->delete_model(model_name)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to delete model: %s", model_name.c_str());
    }

    RCLCPP_INFO(node->get_logger(), "Spawn test completed successfully.");

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
