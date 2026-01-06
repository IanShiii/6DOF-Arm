#include "control/hardware.hpp"

namespace hardware {

    hardware_interface::CallbackReturn HardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
        joint_1_min_degrees_ = info.limits.at("joint_1").min_position;
        joint_1_max_degrees_ = info.limits.at("joint_1").max_position;
        joint_2_min_degrees_ = info.limits.at("joint_2").min_position;
        joint_2_max_degrees_ = info.limits.at("joint_2").max_position;
        joint_3_min_degrees_ = info.limits.at("joint_3").min_position;
        joint_3_max_degrees_ = info.limits.at("joint_3").max_position;
        joint_4_min_degrees_ = info.limits.at("joint_4").min_position;
        joint_4_max_degrees_ = info.limits.at("joint_4").max_position;
        joint_5_min_degrees_ = info.limits.at("joint_5").min_position;
        joint_5_max_degrees_ = info.limits.at("joint_5").max_position;
        joint_6_min_degrees_ = info.limits.at("joint_6").min_position;
        joint_6_max_degrees_ = info.limits.at("joint_6").max_position;

        for (auto & joint : info.joints) {
            if (joint.name == "joint_1") {
                joint_1_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                joint_1_inverted_ = joint.parameters.at("inverted") == "true";
            } else if (joint.name == "joint_2") {
                joint_2_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                joint_2_inverted_ = joint.parameters.at("inverted") == "true";
            } else if (joint.name == "joint_3") {
                joint_3_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                joint_3_inverted_ = joint.parameters.at("inverted") == "true";
            } else if (joint.name == "joint_4") {
                joint_4_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                joint_4_inverted_ = joint.parameters.at("inverted") == "true";
            } else if (joint.name == "joint_5") {
                joint_5_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                joint_5_inverted_ = joint.parameters.at("inverted") == "true";
            } else if (joint.name == "joint_6") {
                joint_6_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                joint_6_inverted_ = joint.parameters.at("inverted") == "true";
            } else if (joint.name == "gripper") {
                gripper_servo_id_ = std::stoi(joint.parameters.at("servo_id"));
                gripper_closed_degrees_ = std::stod(joint.parameters.at("closed_angle"));
                gripper_open_degrees_ = std::stod(joint.parameters.at("open_angle"));
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HardwareInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        joint_1_target_degrees_ = 0.0;
        joint_2_target_degrees_ = 0.0;
        joint_3_target_degrees_ = 0.0;
        joint_4_target_degrees_ = 0.0;
        joint_5_target_degrees_ = 0.0;
        joint_6_target_degrees_ = 0.0;
        gripper_closed_ = false;

        pca.set_pwm_freq(50);
        pca.set_all_pwm(0, 4096);
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HardwareInterface::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HardwareInterface::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HardwareInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HardwareInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        pca.set_all_pwm(0, 4096);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HardwareInterface::on_error([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // --------------------------------
    // SystemInterface overrides
    // --------------------------------

    std::vector<hardware_interface::StateInterface> HardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface("joint_1", hardware_interface::HW_IF_POSITION, &joint_1_target_degrees_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("joint_2", hardware_interface::HW_IF_POSITION, &joint_2_target_degrees_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("joint_3", hardware_interface::HW_IF_POSITION, &joint_3_target_degrees_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("joint_4", hardware_interface::HW_IF_POSITION, &joint_4_target_degrees_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("joint_5", hardware_interface::HW_IF_POSITION, &joint_5_target_degrees_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("joint_6", hardware_interface::HW_IF_POSITION, &joint_6_target_degrees_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("gripper", "closed", &gripper_closed_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_1", hardware_interface::HW_IF_POSITION, &joint_1_target_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_2", hardware_interface::HW_IF_POSITION, &joint_2_target_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_3", hardware_interface::HW_IF_POSITION, &joint_3_target_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_4", hardware_interface::HW_IF_POSITION, &joint_4_target_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_5", hardware_interface::HW_IF_POSITION, &joint_5_target_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_6", hardware_interface::HW_IF_POSITION, &joint_6_target_degrees_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("gripper", "closed", &gripper_closed_));
        return command_interfaces;
    }

    hardware_interface::return_type HardwareInterface::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HardwareInterface::write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        clamp_command_values();

        pca.set_pwm(joint_1_servo_id_, 0, angle_to_ticks(joint_1_target_degrees_, joint_1_inverted_));
        pca.set_pwm(joint_2_servo_id_, 0, angle_to_ticks(joint_2_target_degrees_, joint_2_inverted_));
        pca.set_pwm(joint_3_servo_id_, 0, angle_to_ticks(joint_3_target_degrees_, joint_3_inverted_));
        pca.set_pwm(joint_4_servo_id_, 0, angle_to_ticks(joint_4_target_degrees_, joint_4_inverted_));
        pca.set_pwm(joint_5_servo_id_, 0, angle_to_ticks(joint_5_target_degrees_, joint_5_inverted_));
        pca.set_pwm(joint_6_servo_id_, 0, angle_to_ticks(joint_6_target_degrees_, joint_6_inverted_));

        if (static_cast<bool>(gripper_closed_)) {
            pca.set_pwm(gripper_servo_id_, 0, angle_to_ticks(gripper_closed_degrees_, false));
        } else {
            pca.set_pwm(gripper_servo_id_, 0, angle_to_ticks(gripper_open_degrees_, false));
        }

        return hardware_interface::return_type::OK;
    }

    unsigned int HardwareInterface::angle_to_ticks(double angle, bool inverted) {
        if (inverted) {
            angle *= -1.0;
        }
        double microseconds = 500 + ((angle + 2.35619) / 4.71238) * 2000.0; // 500us to 2500us
        double microseconds_per_tick = 20000.0 / 4096.0; // 20ms period, 4096 ticks
        return (unsigned int)(microseconds / microseconds_per_tick);
    }

    void HardwareInterface::clamp_command_values() {
        joint_1_target_degrees_ = std::clamp(joint_1_target_degrees_, -135.0, 135.0);
        joint_2_target_degrees_ = std::clamp(joint_2_target_degrees_, -135.0, 135.0);
        joint_3_target_degrees_ = std::clamp(joint_3_target_degrees_, -135.0, 135.0);
        joint_4_target_degrees_ = std::clamp(joint_4_target_degrees_, -135.0, 135.0);
        joint_5_target_degrees_ = std::clamp(joint_5_target_degrees_, -135.0, 135.0);
        joint_6_target_degrees_ = std::clamp(joint_6_target_degrees_, -135.0, 135.0);
    }
}

PLUGINLIB_EXPORT_CLASS(hardware::HardwareInterface, hardware_interface::SystemInterface)
