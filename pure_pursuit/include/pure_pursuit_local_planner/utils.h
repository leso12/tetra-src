/* #pragma once
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

namespace pure_pursuit_local_planner {

// plan(보통 "map") -> target_frame(예: "base_footprint" 또는 "odom")로 변환
bool getXPoseTF2(tf2_ros::Buffer& tf_buffer,
                 const std::vector<geometry_msgs::PoseStamped>& plan,
                 const std::string& target_frame,
                 std::vector<geometry_msgs::PoseStamped>& transformed_out,
                 int max_poses);

} // namespace
