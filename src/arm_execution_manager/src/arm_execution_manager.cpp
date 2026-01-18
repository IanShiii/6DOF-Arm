#include "arm_execution_manager/arm_execution_manager.hpp"

ArmExecutionManager::ArmExecutionManager() : Node("arm_execution_manager") {
    target_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", 1, std::bind(&ArmExecutionManager::target_pose_callback, this, std::placeholders::_1));
    rclcpp::sleep_for(std::chrono::seconds(5));
}

void ArmExecutionManager::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    plan_and_execute_to_pose(*msg);
}

void ArmExecutionManager::plan_and_execute_to_pose(const geometry_msgs::msg::PoseStamped &target_pose) {
    move_group_->setPoseTarget(target_pose);
    move_group_->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing plan.");
        move_group_->execute(plan);
        RCLCPP_INFO(this->get_logger(), "Plan executed");
    } else {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
    }
}

void ArmExecutionManager::initialize_move_group() {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    move_group_->setPlanningTime(1.0);
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
    move_group_->setGoalPositionTolerance(0.02);
    move_group_->setGoalOrientationTolerance(0.1);
    move_group_->startStateMonitor();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmExecutionManager>();
    node->initialize_move_group();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


