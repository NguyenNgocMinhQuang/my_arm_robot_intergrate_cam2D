#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "my_robot_hardware/stepper_driver.hpp"

namespace arm_hardware {

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Đảm bảo đóng cổng serial dù node tắt kiểu gì (crash, ctrl+C, ...)
    ~ArmHardwareInterface() override;

    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<StepperDriver> driver_;
    std::string serial_port_;
    int baud_rate_;

}; // class ArmHardwareInterface

} // namespace arm_hardware

#endif