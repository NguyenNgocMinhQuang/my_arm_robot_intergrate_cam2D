#include "my_robot_hardware/arm_hardware_interface.hpp"

#include <cmath>

namespace arm_hardware {

ArmHardwareInterface::~ArmHardwareInterface()
{
    // Bảo đảm cổng serial luôn được đóng, kể cả khi node bị kill giữa chừng
    if (driver_) {
        driver_->closePort();
    }
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    serial_port_ = info_.hardware_parameters["serial_port"];
    baud_rate_   = std::stoi(info_.hardware_parameters["baud_rate"]);

    driver_ = std::make_shared<StepperDriver>(serial_port_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init(baud_rate_) != 0) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    // Động cơ bước là open-loop (không có encoder phản hồi), nên không thể
    // "đọc" vị trí thật lúc khởi động. Quy ước: coi vị trí hiện tại của
    // cả 6 khớp luôn là gốc 0.0 khi kích hoạt (giả định tay đã ở home).
    set_state("joint1/position", 0.0);
    set_state("joint2/position", 0.0);
    set_state("joint3/position", 0.0);
    set_state("joint4/position", 0.0);
    set_state("joint5/position", 0.0);
    set_state("joint6/position", 0.0);
    set_state("gripper_control/position", 0.0);

    // Không có "activateWithPositionMode" như Dynamixel: động cơ bước +
    // driver STM32 không cần lệnh bật chế độ, MCU tự mặc định sẵn.

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    // Không có lệnh "deactivate/disable torque" cho stepper ở tầng driver
    // này; nếu MCU của bạn có lệnh dừng khẩn cấp riêng, gọi driver_->... ở đây.
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // Open-loop state mirror: không có encoder thật, nên "trạng thái" chỉ
    // là copy lại command gần nhất của CẢ 6 khớp, để MoveIt/joint_state_broadcaster
    // nghĩ rằng khớp đã tới đúng vị trí đã yêu cầu. Giữ nguyên state cũ nếu
    // command hiện đang là NaN (chưa có lệnh nào được gửi cho khớp đó).
    const char * joint_names[7] = {
        "joint1/position", "joint2/position", "joint3/position",
        "joint4/position", "joint5/position", "joint6/position",
        "gripper_control/position"
    };
    for (const char * joint : joint_names) {
        double cmd = get_command(joint);
        if (!std::isnan(cmd)) {
            set_state(joint, cmd);
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // Chống lỗi NaN (Not a Number) khi 1 trong các nhóm chưa có lệnh MoveIt
    double j1 = std::isnan(get_command("joint1/position")) ? 0.0 : get_command("joint1/position");
    double j2 = std::isnan(get_command("joint2/position")) ? 0.0 : get_command("joint2/position");
    double j3 = std::isnan(get_command("joint3/position")) ? 0.0 : get_command("joint3/position");
    double j4 = std::isnan(get_command("joint4/position")) ? 0.0 : get_command("joint4/position");
    double j5 = std::isnan(get_command("joint5/position")) ? 0.0 : get_command("joint5/position");
    double j6 = std::isnan(get_command("joint6/position")) ? 0.0 : get_command("joint6/position");
    double g1 = std::isnan(get_command("gripper_control/position")) ? 0.0 : get_command("gripper_control/position");

    driver_->sendJointPositions(j1, j2, j3, j4, j5, j6, g1);

    return hardware_interface::return_type::OK;
}

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)