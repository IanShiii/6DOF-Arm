#pragma once

#include <PiPCA9685/PCA9685.h>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/system_interface.hpp"

namespace hardware {
    class HardwareInterface : public hardware_interface::SystemInterface {
        public:
            HardwareInterface() = default;

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::CallbackReturn on_configure([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_error([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::return_type read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) override;
            hardware_interface::return_type write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period) override;

        private:
            PiPCA9685::PCA9685 pca;

            // PCA9685 servo numbers and GPIO Pins

            int joint_1_servo_id_;
            int joint_2_servo_id_;
            int joint_3_servo_id_;
            int joint_4_servo_id_;
            int joint_5_servo_id_;
            int joint_6_servo_id_;

            // Inversions

            bool joint_1_inverted_;
            bool joint_2_inverted_;
            bool joint_3_inverted_;
            bool joint_4_inverted_;
            bool joint_5_inverted_;
            bool joint_6_inverted_;

            // Limits

            double joint_1_min_degrees_;
            double joint_1_max_degrees_;
            double joint_2_min_degrees_;
            double joint_2_max_degrees_;
            double joint_3_min_degrees_;
            double joint_3_max_degrees_;
            double joint_4_min_degrees_;
            double joint_4_max_degrees_;
            double joint_5_min_degrees_;
            double joint_5_max_degrees_;
            double joint_6_min_degrees_;
            double joint_6_max_degrees_;

            // Targets

            double joint_1_target_degrees_;
            double joint_2_target_degrees_;
            double joint_3_target_degrees_;
            double joint_4_target_degrees_;
            double joint_5_target_degrees_;
            double joint_6_target_degrees_;

            /**
             * @brief Converts an angle in radians to ticks for the PCA9685.
             * @param angle Angle in radians [0, 4.71239] (0 to 270 degrees)
             */
            unsigned int angle_to_ticks(double radians, bool inverted);

            bool is_within_limits(double joint_1_target_angle, double joint_2_target_angle, double joint_3_target_angle,
                                  double joint_4_target_angle, double joint_5_target_angle, double joint_6_target_angle);
    };
}
