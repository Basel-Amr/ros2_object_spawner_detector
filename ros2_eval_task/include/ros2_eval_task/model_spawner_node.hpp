/**
 * @file model_spawner_node.hpp
 * @brief Declaration of ModelSpawnerNode class for spawning/deleting Gazebo models,
 *        publishing markers, and saving camera frames.
 *
 * Author: Basel Amr Barakat
 * Email: baselamr52@gmail.com
 */

#pragma once

#include "ros2_eval_task/gazebo_utils_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include <random>
#include <string>
#include <vector>
#include <map>
#include <filesystem>

namespace fs = std::filesystem;

class ModelSpawnerNode : public rclcpp::Node
{
public:
    explicit ModelSpawnerNode(rclcpp::Node::SharedPtr node);

private:
    // === Core callbacks ===
    void timer_callback();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    // === Utilities ===
    void prepare_base_save_dir();
    void prepare_model_session_dir(const std::string &model_name);
    void publish_marker(const std::string &model_name, const geometry_msgs::msg::Pose &pose);

    void load_reference_templates();  // ✅ Added: loads all reference templates

    double rand_double(double min, double max);
    int rand_int(int min, int max);

    // === Members ===
    std::shared_ptr<ros2_eval_task::GazeboUtilsClient> utils_;
    std::vector<std::string> models_;
    std::string current_model_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    std::mt19937 rng_;
    int image_id_;

    std::string save_dir_;             // model-specific session folder
    std::string base_save_dir_;        // base directory for saving
    std::string session_dir_created_;  // timestamp for session folder

    bool warned_get_entity_state_;     // warn only once if get_entity_state unavailable

    std::map<std::string, cv::Mat> templates_;  // stores loaded reference templates
};
