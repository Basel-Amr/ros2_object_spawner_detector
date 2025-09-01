#ifndef ROS2_EVAL_TASK__GAZEBO_UTILS_CLIENT_HPP_
#define ROS2_EVAL_TASK__GAZEBO_UTILS_CLIENT_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include <gazebo_msgs/srv/get_entity_state.hpp>

namespace ros2_eval_task
{

class GazeboUtilsClient
{
public:
  explicit GazeboUtilsClient(rclcpp::Node::SharedPtr node);

  bool spawn_model(const std::string &model_name,
                   const std::string &xml,
                   const geometry_msgs::msg::Pose &pose);

  bool delete_model(const std::string &model_name);

  bool get_entity_pose(const std::string &model_name,
                        geometry_msgs::msg::Pose &pose);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr get_state_client_;
};

}  // namespace ros2_eval_task

#endif  // ROS2_EVAL_TASK__GAZEBO_UTILS_CLIENT_HPP_
