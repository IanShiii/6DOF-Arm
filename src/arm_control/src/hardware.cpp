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
        return command_interfaces;
    }

    hardware_interface::return_type HardwareInterface::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HardwareInterface::write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) {
        if (!is_within_limits(joint_1_target_degrees_, joint_2_target_degrees_, joint_3_target_degrees_, joint_4_target_degrees_, joint_5_target_degrees_, joint_6_target_degrees_)) {
            RCLCPP_ERROR(
                rclcpp::get_logger("HardwareInterface"),
                "One or more joint target angles are out of limits. Command not sent to servos.");
            return hardware_interface::return_type::ERROR;
        }

        pca.set_pwm(joint_1_servo_id_, 0, angle_to_ticks(joint_1_target_degrees_, joint_1_inverted_));
        pca.set_pwm(joint_2_servo_id_, 0, angle_to_ticks(joint_2_target_degrees_, joint_2_inverted_));
        pca.set_pwm(joint_3_servo_id_, 0, angle_to_ticks(joint_3_target_degrees_, joint_3_inverted_));
        pca.set_pwm(joint_4_servo_id_, 0, angle_to_ticks(joint_4_target_degrees_, joint_4_inverted_));
        pca.set_pwm(joint_5_servo_id_, 0, angle_to_ticks(joint_5_target_degrees_, joint_5_inverted_));
        pca.set_pwm(joint_6_servo_id_, 0, angle_to_ticks(joint_6_target_degrees_, joint_6_inverted_));

        return hardware_interface::return_type::OK;
    }

    unsigned int HardwareInterface::angle_to_ticks(double angle, bool inverted) {
        if (inverted) {
            angle = 4.71239 - angle;
        }
        double microseconds = 500 + (angle / 4.71239 * 2000.0); // 500us to 2500us
        double microseconds_per_tick = 20000.0 / 4096.0; // 20ms period, 4096 ticks
        return (unsigned int)(microseconds / microseconds_per_tick);
    }

    bool HardwareInterface::is_within_limits(double joint_1_target_angle, double joint_2_target_angle, double joint_3_target_angle,
                          double joint_4_target_angle, double joint_5_target_angle, double joint_6_target_angle) {
        return (joint_1_target_angle >= joint_1_min_degrees_ && joint_1_target_angle <= joint_1_max_degrees_) &&
               (joint_2_target_angle >= joint_2_min_degrees_ && joint_2_target_angle <= joint_2_max_degrees_) &&
               (joint_3_target_angle >= joint_3_min_degrees_ && joint_3_target_angle <= joint_3_max_degrees_) &&
               (joint_4_target_angle >= joint_4_min_degrees_ && joint_4_target_angle <= joint_4_max_degrees_) &&
               (joint_5_target_angle >= joint_5_min_degrees_ && joint_5_target_angle <= joint_5_max_degrees_) &&
               (joint_6_target_angle >= joint_6_min_degrees_ && joint_6_target_angle <= joint_6_max_degrees_);
    }
}

PLUGINLIB_EXPORT_CLASS(hardware::HardwareInterface, hardware_interface::SystemInterface)
