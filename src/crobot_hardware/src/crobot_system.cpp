#include "crobot_hardware/crobot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crobot_hardware
{
    hardware_interface::CallbackReturn CrobotHardware::on_init(
        const hardware_interface::HardwareInfo *& info
    )
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS
        )
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        
    }
}