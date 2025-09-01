/**
 * @file gazebo_utils_client.cpp
 * @brief Utility class for interacting with Gazebo services in ROS2.
 * 
 * Provides methods to spawn, delete, and query models in Gazebo using the
 * /spawn_entity, /delete_entity, and /get_entity_state services.
 * 
 * Author: Basel Amr Barakat
 * Email: baselamr52@gmail.com
 */

#include "ros2_eval_task/gazebo_utils_client.hpp"

using namespace std::chrono_literals;

namespace ros2_eval_task
{

/**
 * @brief Construct a new GazeboUtilsClient object
 * 
 * @param node Shared pointer to a ROS2 node
 */
GazeboUtilsClient::GazeboUtilsClient(rclcpp::Node::SharedPtr node)
: node_(node)
{
    // Initialize clients for Gazebo services
    spawn_client_ = node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    delete_client_ = node_->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    get_state_client_ = node_->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
}

/**
 * @brief Spawn a model in Gazebo and verify it exists
 * 
 * @param model_name Name of the model to spawn
 * @param xml URDF/SDF XML string of the model
 * @param pose Initial pose of the model in the Gazebo world
 * @return true if model was spawned and verified successfully
 * @return false otherwise
 */
bool GazeboUtilsClient::spawn_model(const std::string &model_name,
                                    const std::string &xml,
                                    const geometry_msgs::msg::Pose &pose)
{
    // Wait for the spawn service to be available
    if (!spawn_client_->wait_for_service(2s)) {
        RCLCPP_ERROR(node_->get_logger(), "❌ SpawnEntity service not available");
        return false;
    }

    // Prepare the request
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = model_name;
    request->xml = xml;
    request->initial_pose = pose;

    // Send the request asynchronously and wait for the response
    auto future = spawn_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to call SpawnEntity service");
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Gazebo failed to spawn model: %s", response->status_message.c_str());
        return false;
    }

    // Verify with get_entity_state
    geometry_msgs::msg::Pose actual_pose;
    if (get_entity_pose(model_name, actual_pose)) {
        RCLCPP_INFO(node_->get_logger(),
            "✅ Model '%s' spawned successfully at (x=%.2f, y=%.2f, z=%.2f)",
            model_name.c_str(),
            actual_pose.position.x,
            actual_pose.position.y,
            actual_pose.position.z);
        return true;
    }

    RCLCPP_WARN(node_->get_logger(),
        "⚠️ Model '%s' was spawned, but could not verify with /get_entity_state.",
        model_name.c_str());
    return false;
}

/**
 * @brief Delete a model from Gazebo
 * 
 * @param model_name Name of the model to delete
 * @return true if model was deleted successfully
 * @return false otherwise
 */
bool GazeboUtilsClient::delete_model(const std::string &model_name)
{
    if (!delete_client_->wait_for_service(2s)) {
        RCLCPP_ERROR(node_->get_logger(), "❌ DeleteEntity service not available");
        return false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = model_name;

    auto future = delete_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to call DeleteEntity service");
        return false;
    }

    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(node_->get_logger(), "🗑️ Deleted model: %s", model_name.c_str());
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to delete model '%s': %s",
                     model_name.c_str(), response->status_message.c_str());
        return false;
    }
}

/**
 * @brief Get the pose of a model using /get_entity_state
 * 
 * @param model_name Name of the model to query
 * @param pose Reference to store the returned pose
 * @return true if the model exists and pose was retrieved
 * @return false otherwise
 */
bool GazeboUtilsClient::get_entity_pose(const std::string &model_name,
                                        geometry_msgs::msg::Pose &pose)
{
    if (!get_state_client_->wait_for_service(2s)) {
        RCLCPP_ERROR(node_->get_logger(), "❌ GetEntityState service not available");
        return false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = model_name;
    request->reference_frame = "world";

    auto future = get_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to call GetEntityState service");
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Model '%s' does not exist in Gazebo", model_name.c_str());
        return false;
    }

    pose = response->state.pose;
    return true;
}

}  // namespace ros2_eval_task
