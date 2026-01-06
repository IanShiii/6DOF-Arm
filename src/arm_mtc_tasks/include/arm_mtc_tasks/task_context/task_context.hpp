#pragma once

#include <string>
#include <memory>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

struct TaskContext {
  std::string arm_group;
  std::string gripper_group;
  std::string eef;
  std::string ik_frame;

  moveit::task_constructor::solvers::PlannerInterfacePtr pipeline;
  moveit::task_constructor::solvers::PlannerInterfacePtr cartesian;
};
