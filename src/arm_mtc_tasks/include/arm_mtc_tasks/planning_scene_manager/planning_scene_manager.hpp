#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PlanningSceneManager {
    public:
        PlanningSceneManager(const rclcpp::Node::SharedPtr& node);

        /**
         * @brief Adds static environment objects to the planning scene (floor and reef)
         */
        void add_static_environment();

        void spawn_unplaced_coral();
    private:
        rclcpp::Node::SharedPtr node_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

        moveit_msgs::msg::CollisionObject makeCoralCollisionObject(
            const std::string& id,
            const geometry_msgs::msg::PoseStamped& pose
        );
};
