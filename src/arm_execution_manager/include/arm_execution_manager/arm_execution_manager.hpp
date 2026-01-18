#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <atomic>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArmExecutionManager : public rclcpp::Node {
    public:
        ArmExecutionManager();
        void initialize_move_group();

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

        void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        bool is_within_tolerance_to_current_pose(const geometry_msgs::msg::PoseStamped &target);
        void plan_and_execute_to_pose(const geometry_msgs::msg::PoseStamped &target);
};
