#include "arm_mtc_tasks/planning_scene_manager/planning_scene_manager.hpp"

PlanningSceneManager::PlanningSceneManager(const rclcpp::Node::SharedPtr& node) {
    this->node_ = node;
    this->planning_scene_interface_ = moveit::planning_interface::PlanningSceneInterface();
}

void PlanningSceneManager::add_static_environment() {
    // Implementation for adding static environment objects (floor and reef)
}

void PlanningSceneManager::spawn_unplaced_coral() {
    geometry_msgs::msg::PoseStamped pickup_pose;
    pickup_pose.header.frame_id = "world";
    pickup_pose.pose.position.x = 1.0;
    pickup_pose.pose.position.y = 0.0;
    pickup_pose.pose.position.z = 0.5;
    pickup_pose.pose.orientation.w = 1.0;

    moveit_msgs::msg::CollisionObject coral = makeCoralCollisionObject("unplaced_coral", pickup_pose);
    planning_scene_interface_.applyCollisionObject(coral);
}

moveit_msgs::msg::CollisionObject PlanningSceneManager::makeCoralCollisionObject(const std::string& id, const geometry_msgs::msg::PoseStamped& pose) {
    moveit_msgs::msg::CollisionObject coral;
    coral.id = id;
    coral.header = pose.header;
    coral.pose = pose.pose;
    
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions = {0.2, 0.1}; // (height, radius)

    coral.primitives.push_back(cylinder);
    coral.primitive_poses.push_back(pose.pose);
    coral.operation = moveit_msgs::msg::CollisionObject::ADD;

    return coral;
}


