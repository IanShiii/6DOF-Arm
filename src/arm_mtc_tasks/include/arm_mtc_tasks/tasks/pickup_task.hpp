#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <string>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/container.h>

#include "arm_mtc_tasks/task_context/task_context.hpp"

namespace arm_tasks {

    /**
     * Creates a pickup task using MoveIt Task Constructor.
     *
     * @param node            ROS node (used for MoveIt context, logging, parameters)
     * @param context         TaskContext containing configuration for the task
     * @param object_id       Planning scene object to pick
     * @param support_surface Surface the object rests on (may be empty)
     */
    moveit::task_constructor::Task create_pickup_task(
        const rclcpp::Node::SharedPtr& node,
        const TaskContext& context,
        const std::string& object_id,
        const std::string& support_surface
    );

}  // namespace tasks
