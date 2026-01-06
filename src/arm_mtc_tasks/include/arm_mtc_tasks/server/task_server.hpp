#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm_mtc_tasks/action/manipulation.hpp"
#include "arm_mtc_tasks/tasks/pickup_task.hpp"
#include "arm_mtc_tasks/tasks/score_task.hpp"
#include "arm_mtc_tasks/tasks/stow_task.hpp"
#include "arm_mtc_tasks/task_context/task_context.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>

class TaskServer : public rclcpp::Node {
  public:
    TaskServer();
    void init_task_context();

  private:
    rclcpp_action::Server<arm_mtc_tasks::action::Manipulation>::SharedPtr server_;
    TaskContext context_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const arm_mtc_tasks::action::Manipulation::Goal>);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_mtc_tasks::action::Manipulation>>);
    void handle_accept(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_mtc_tasks::action::Manipulation>> goal);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_mtc_tasks::action::Manipulation>> goal);
};
