#include "arm_mtc_tasks/server/task_server.hpp"

TaskServer::TaskServer() : rclcpp::Node("task_server") {
    server_ = rclcpp_action::create_server<arm_mtc_tasks::action::Manipulation>(
        this,
        "task",
        std::bind(&TaskServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&TaskServer::handle_accept, this, std::placeholders::_1)
    );
}

void TaskServer::init_task_context() {
    context_.arm_group = "arm";
    context_.gripper_group = "gripper";
    context_.eef = "gripper";
    context_.ik_frame = "end_effector_fixed_link";

    context_.pipeline = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_from_this());
    context_.pipeline->setMaxAccelerationScalingFactor(0.5);
    context_.pipeline->setMaxVelocityScalingFactor(0.5);

    context_.cartesian = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
    context_.cartesian->setMaxVelocityScalingFactor(0.5);
    context_.cartesian->setMaxAccelerationScalingFactor(0.5);
}

rclcpp_action::GoalResponse TaskServer::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const arm_mtc_tasks::action::Manipulation::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with command %d", goal->command);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskServer::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_mtc_tasks::action::Manipulation>>) {\
    // No cancelation support for now
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskServer::handle_accept(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_mtc_tasks::action::Manipulation>> goal) {
    std::thread{[this, goal]() {
        execute(goal);
    }}.detach();
}

void TaskServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_mtc_tasks::action::Manipulation>> goal) {
    auto result = std::make_shared<arm_mtc_tasks::action::Manipulation::Result>();
    bool success = false;

    switch (goal->get_goal()->command) {
        case arm_mtc_tasks::action::Manipulation::Goal::PICKUP: {
            auto task = arm_tasks::create_pickup_task(shared_from_this(), context_, "coral", "floor");
            RCLCPP_INFO(this->get_logger(), "Created PICKUP task");
            if (task.plan() == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "PICKUP task planned successfully, executing...");
                if (task.execute(*task.solutions().front()) == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "PICKUP task executed successfully");
                    success = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "PICKUP task execution failed");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "PICKUP task planning failed");
            }
            break;
        }
    }

    // moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "arm");
    
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.orientation.w = 1.0;
    // target_pose.position.x = 0.5;
    // target_pose.position.y = 0.5;
    // target_pose.position.z = 0.5;

    // move_group.setPoseTarget(target_pose);
    // move_group.setGoalOrientationTolerance(0.5);
    // move_group.setGoalPositionTolerance(0.1);

    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (success) {
    //     RCLCPP_INFO(this->get_logger(), "Plan found, executing...");
    //     success = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //     if (success) {
    //         RCLCPP_INFO(this->get_logger(), "Motion executed successfully");
    //     } else {
    //         RCLCPP_ERROR(this->get_logger(), "Motion execution failed");
    //     }
    // } else {
    //     RCLCPP_ERROR(this->get_logger(), "Planning failed");
    // }
    

    result->success = success;
    goal->succeed(result);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TaskServer>();
  node->init_task_context();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
