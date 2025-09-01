/**
 * @file model_spawner_node.cpp
 * @brief ROS2 node to randomly spawn/delete Gazebo models, publish markers,
 *        run template matching detection, and save camera frames.
 *
 * Author: Basel Amr Barakat
 * Email: baselamr52@gmail.com
 */

#include "ros2_eval_task/model_spawner_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <chrono>
#include <filesystem>
#include <sys/stat.h>
#include <iomanip>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

// === Constructor ===
ModelSpawnerNode::ModelSpawnerNode(rclcpp::Node::SharedPtr node)
: Node("model_spawner_node"), rng_(std::random_device{}()), image_id_(0), warned_get_entity_state_(false)
{
    // Initialize Gazebo utility client
    utils_ = std::make_shared<ros2_eval_task::GazeboUtilsClient>(node);

    // Define models
    models_ = {"battery_9v_leader", "battery_energizer", "lipo_battery", "battery_varita"};

    // Publisher for markers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("spawned_markers", 10);

    // Subscribe to camera topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ModelSpawnerNode::image_callback, this, std::placeholders::_1));

    // Prepare save dir
    prepare_base_save_dir();

    // Load templates for detection
    load_reference_templates();

    // Timer (spawn every 5s)
    timer_ = this->create_wall_timer(5s, std::bind(&ModelSpawnerNode::timer_callback, this));
}

// === Private Methods ===
void ModelSpawnerNode::prepare_base_save_dir()
{
    const char *home = std::getenv("HOME");
    std::string candidate;

    if (home && std::strlen(home) > 0) {
        candidate = std::string(home) + "/ros2_ws/src/ros2_eval_task/captured_images";
    } else {
        candidate = "/tmp/ros2_captured_images";
        RCLCPP_WARN(this->get_logger(), "HOME not found, using fallback: %s", candidate.c_str());
    }

    try {
        fs::create_directories(candidate);
    } catch (const fs::filesystem_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create save dir '%s': %s", candidate.c_str(), e.what());
        candidate = "/tmp/ros2_captured_images";
        fs::create_directories(candidate);
    }

    // Check writability
    std::string test_file = candidate + "/.perm_test";
    FILE *f = std::fopen(test_file.c_str(), "w");
    if (f) {
        std::fputs("test", f);
        std::fclose(f);
        std::remove(test_file.c_str());
        base_save_dir_ = candidate;
        save_dir_ = candidate;
        RCLCPP_INFO(this->get_logger(), "📂 Images will be saved to: %s", save_dir_.c_str());
    } else {
        base_save_dir_ = "/tmp/ros2_captured_images";
        save_dir_ = base_save_dir_;
        fs::create_directories(save_dir_);
        RCLCPP_WARN(this->get_logger(), "Permission denied. Using fallback: %s", save_dir_.c_str());
    }
}

void ModelSpawnerNode::timer_callback()
{
    if (!current_model_.empty()) {
        utils_->delete_model(current_model_);
    }

    current_model_ = models_[rand_int(0, static_cast<int>(models_.size()) - 1)];

    geometry_msgs::msg::Pose pose;
    pose.position.x = rand_double(-0.21, 0.21);
    pose.position.y = rand_double(-0.43, 0.43);
    pose.position.z = 1.1;
    pose.orientation.w = 1.0;

    std::string pkg_share;
    try {
        pkg_share = ament_index_cpp::get_package_share_directory("ros2_eval_task");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to locate package: %s", e.what());
        return;
    }

    std::string sdf_path = pkg_share + "/models/" + current_model_ + "/model.sdf";
    std::ifstream ifs(sdf_path);
    if (!ifs) {
        RCLCPP_ERROR(this->get_logger(), "❌ Could not open: %s", sdf_path.c_str());
        return;
    }
    std::stringstream buffer;
    buffer << ifs.rdbuf();

    if (utils_->spawn_model(current_model_, buffer.str(), pose)) {
        geometry_msgs::msg::Pose actual_pose;
        if (utils_->get_entity_pose(current_model_, actual_pose)) {
            RCLCPP_INFO(this->get_logger(), "🎯 Spawned '%s' at (%.2f, %.2f, %.2f)",
                        current_model_.c_str(), actual_pose.position.x, actual_pose.position.y, actual_pose.position.z);
            publish_marker(current_model_, actual_pose);
            prepare_model_session_dir(current_model_);
        } else {
            if (!warned_get_entity_state_) {
                RCLCPP_WARN(this->get_logger(), "⚠️ get_entity_state unavailable for '%s'", current_model_.c_str());
                warned_get_entity_state_ = true;
            }
            publish_marker(current_model_, pose);
            prepare_model_session_dir(current_model_);
        }
    }
}

void ModelSpawnerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (save_dir_.empty() || save_dir_ == "/") return;

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;

        std::string detected_model = "unknown";
        double best_score = -1.0;
        double second_best_score = -1.0;
        cv::Rect best_bbox;

        const double global_threshold = 0.60; // require at least this score
        const double margin_threshold = 0.08; // best must exceed second best by margin

        for (const auto &entry : templates_) {
            const std::string &model_name = entry.first;
            const cv::Mat &templ = entry.second;

            if (frame.cols < templ.cols || frame.rows < templ.rows) continue;

            cv::Mat result;
            cv::matchTemplate(frame, templ, result, cv::TM_CCOEFF_NORMED);

            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

            if (maxVal > best_score) {
                second_best_score = best_score;
                best_score = maxVal;
                detected_model = model_name;
                best_bbox = cv::Rect(maxLoc.x, maxLoc.y, templ.cols, templ.rows);
            } else if (maxVal > second_best_score) {
                second_best_score = maxVal;
            }
        }

        // Draw if above thresholds
        if (best_score >= global_threshold && (best_score - second_best_score) >= margin_threshold) {
            cv::rectangle(frame, best_bbox, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, detected_model, best_bbox.tl() + cv::Point(0, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // Save in model folder
            std::string model_dir = base_save_dir_ + "/" + detected_model;
            fs::create_directories(model_dir);

            auto ts = std::chrono::system_clock::now().time_since_epoch().count();
            std::ostringstream oss;
            oss << model_dir << "/detected_" << image_id_++ << "_" << ts << ".png";
            cv::imwrite(oss.str(), frame);

            RCLCPP_INFO(this->get_logger(), "✅ Image saved with detection: %s | Model: %s (score=%.2f)",
                        oss.str().c_str(), detected_model.c_str(), best_score);
        } else {
            // Save in "unknown"
            std::string model_dir = base_save_dir_ + "/unknown";
            fs::create_directories(model_dir);

            auto ts = std::chrono::system_clock::now().time_since_epoch().count();
            std::ostringstream oss;
            oss << model_dir << "/detected_" << image_id_++ << "_" << ts << ".png";
            cv::imwrite(oss.str(), frame);

            RCLCPP_INFO(this->get_logger(), "⚠️ Detection uncertain, saved as unknown: %s (score=%.2f)",
                        oss.str().c_str(), best_score);
        }

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Image processing error: %s", e.what());
    }
}

void ModelSpawnerNode::publish_marker(const std::string &model_name, const geometry_msgs::msg::Pose &pose)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "models";
    marker.id = rand_int(0, 1000000);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;

    if (model_name == "battery_9v_leader") {
        marker.color.r = 1.0; marker.color.a = 1.0;
    } else if (model_name == "battery_energizer") {
        marker.color.g = 1.0; marker.color.a = 1.0;
    } else if (model_name == "lipo_battery") {
        marker.color.b = 1.0; marker.color.a = 1.0;
    }

    marker_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "📍 Marker published for: %s", model_name.c_str());
}

void ModelSpawnerNode::prepare_model_session_dir(const std::string &model_name)
{
    if (session_dir_created_.empty()) {
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ts;
        ts << std::put_time(std::localtime(&t), "%Y-%m-%d_%H-%M-%S");
        session_dir_created_ = ts.str();
    }

    std::string model_dir = base_save_dir_ + "/" + model_name + "/" + session_dir_created_;
    fs::create_directories(model_dir);
    save_dir_ = model_dir;
}

void ModelSpawnerNode::load_reference_templates()
{
    templates_.clear();
    std::string pkg_share = ament_index_cpp::get_package_share_directory("ros2_eval_task");
    std::string templates_dir = pkg_share + "/templates";

    for (const auto &model_name : models_) {
        std::string path = templates_dir + "/" + model_name + ".png";
        cv::Mat templ = cv::imread(path, cv::IMREAD_COLOR);
        if (!templ.empty()) {
            templates_[model_name] = templ;
            RCLCPP_INFO(this->get_logger(), "📥 Loaded template for: %s", model_name.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Could not load template: %s", path.c_str());
        }
    }
}

double ModelSpawnerNode::rand_double(double min, double max)
{
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng_);
}

int ModelSpawnerNode::rand_int(int min, int max)
{
    std::uniform_int_distribution<int> dist(min, max);
    return dist(rng_);
}

// === Main ===
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto base_node = std::make_shared<rclcpp::Node>("model_spawner_node_base");
    auto spawner = std::make_shared<ModelSpawnerNode>(base_node);
    rclcpp::spin(spawner);
    rclcpp::shutdown();
    return 0;
}
