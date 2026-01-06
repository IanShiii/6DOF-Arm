#include "arm_mtc_tasks/tasks/pickup_task.hpp"

namespace arm_tasks {

  // moveit::task_constructor::Task create_pickup_task(
  //   const rclcpp::Node::SharedPtr& node,
  //   const TaskContext& context,
  //   const std::string& object_id,
  //   const std::string& support_surface)
  // {
  //   moveit::task_constructor::Task task;
  //   task.stages()->setName("pickup");
  //   task.loadRobotModel(node);

  //   task.setProperty("group", context.arm_group);
  //   task.setProperty("eef", context.eef);
  //   task.setProperty("ik_frame", context.ik_frame);

  //   // Current state
  //   task.add(std::make_unique<moveit::task_constructor::stages::CurrentState>("current"));

  //   // Open gripper
  //   auto open = std::make_unique<moveit::task_constructor::stages::MoveTo>("open gripper", context.pipeline);
  //   open->setGroup("gripper");
  //   open->setGoal("open");
  //   task.add(std::move(open));

  //   // Go to Pre-Grasp pose
  //   auto go_to_pre_grasp = std::make_unique<moveit::task_constructor::stages::MoveTo>("pre-grasp", context.pipeline);

  //   geometry_msgs::msg::PoseStamped pre_grasp_pose;
  //   pre_grasp_pose.header.frame_id = "world";
  //   pre_grasp_pose.pose.position.x = 0.5;
  //   pre_grasp_pose.pose.position.y = 0.0;
  //   pre_grasp_pose.pose.position.z = 0.4;
  //   pre_grasp_pose.pose.orientation.w = 1.0;

  //   go_to_pre_grasp->setGroup("arm");
  //   go_to_pre_grasp->setGoal(pre_grasp_pose);
  //   task.add(std::move(go_to_pre_grasp));


  //   // Go to Grasp pose
  //   auto go_to_grasp = std::make_unique<moveit::task_constructor::stages::MoveTo>("approach", context.cartesian);

  //   geometry_msgs::msg::PoseStamped grasp_target;

  //   grasp_target.header.frame_id = "world";
  //   grasp_target.pose.position.x = 0.5;
  //   grasp_target.pose.position.y = 0.0;
  //   grasp_target.pose.position.z = 0.2;
  //   grasp_target.pose.orientation.w = 1.0;

  //   go_to_grasp->setGroup("arm");
  //   go_to_grasp->setGoal(grasp_target);
  //   task.add(std::move(go_to_grasp));

  //   // Close gripper
  //   auto close = std::make_unique<moveit::task_constructor::stages::MoveTo>("close gripper", context.pipeline);
  //   close->setGroup("gripper");
  //   close->setGoal("closed");
  //   task.add(std::move(close));

  //   // Attach object
  //   // auto attach = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("attach object");
  //   // attach->attachObject(object_id, "end_effector_fixed_link");

  //   // if (!support_surface.empty()) {
  //   //   attach->setProperty("support_surface", support_surface);
  //   // }

  //   // task.add(std::move(attach));

  //   // Lift
  //   auto lift = std::make_unique<moveit::task_constructor::stages::MoveRelative>("lift", context.cartesian);
  //   lift->setGroup("arm");

  //   geometry_msgs::msg::Vector3Stamped lift_dir;
  //   lift_dir.header.frame_id = "world";
  //   lift_dir.vector.z = 1.0;

  //   lift->setDirection(lift_dir);
  //   lift->setMinMaxDistance(0.05, 0.10);
  //   task.add(std::move(lift));

  //   return task;
  // }

  moveit::task_constructor::Task create_pickup_task(
    const rclcpp::Node::SharedPtr& node,
    const TaskContext& context,
    const std::string& object_id,
    const std::string& support_surface)
  {
    moveit::task_constructor::Task task;
    task.setName("pickup");
    task.loadRobotModel(node);

    task.setProperty("group", context.arm_group);
    task.setProperty("eef", context.eef);
    task.setProperty("ik_frame", context.ik_frame);

    // Current state
    task.add(std::make_unique<moveit::task_constructor::stages::CurrentState>("current"));

    // Go to Grasp pose
    auto go_to_grasp = std::make_unique<moveit::task_constructor::stages::MoveTo>("approach", context.pipeline);

    geometry_msgs::msg::PoseStamped grasp_target;

    grasp_target.header.frame_id = "world";
    grasp_target.pose.position.x = 0.5;
    grasp_target.pose.position.y = 0.0;
    grasp_target.pose.position.z = 0.2;
    grasp_target.pose.orientation.w = 1.0;

    go_to_grasp->setGroup(context.arm_group);
    go_to_grasp->setIKFrame(context.ik_frame);
    go_to_grasp->setGoal(grasp_target);
    task.add(std::move(go_to_grasp));

    return task;
  }
}  // namespace tasks
