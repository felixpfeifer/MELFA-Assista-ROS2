//
// Created by felix on 05.03.24.
//

#include "melfa_assita_ros2_hw/melfa_hardware_interface.hpp"

CallbackReturn melfa_hardware::MelfaHW::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
        return CallbackReturn::ERROR;
    }
    
    robot_ip = info_.hardware_parameters["robot_ip"];

    joint_position_state_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    joint_position_command_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for(const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        // MELFA Supports only a Position Interface and State for each Joint
        if(joint.command_interfaces.size() != 1){
            RCLCPP_FATAL(logger,"Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(
            logger,
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
        RCLCPP_FATAL(
            logger,
            "Joint '%s' has %d state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(
            logger,
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
        }
    }

    return CallbackReturn::SUCCESS;
    }


std::vector<hardware_interface::StateInterface> melfa_hardware::MelfaHW::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_state_[i]));
        RCLCPP_INFO(logger,"Joint %s", info_.joints[i].name.c_str());
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> melfa_hardware::MelfaHW::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
    }

    return command_interfaces;
}

/**
 * @brief Function to write the command Position to the MELFA Robot
*/
return_type melfa_hardware::MelfaHW::write(const rclcpp::Time &, const rclcpp::Duration &) {
    return return_type::OK;
}

return_type melfa_hardware::MelfaHW::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    return return_type::OK;
}

/**
 * @brief Function to initialize the Communication with the MELFA Robot
*/
CallbackReturn melfa_hardware::MelfaHW::on_configure(const rclcpp_lifecycle::State &previous_state) {
    return CallbackReturn::SUCCESS;
}

CallbackReturn melfa_hardware::MelfaHW::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/){
    return CallbackReturn::SUCCESS;

}


CallbackReturn melfa_hardware::MelfaHW::on_activate(const rclcpp_lifecycle::State &previous_state) {
    return CallbackReturn::SUCCESS;
}



CallbackReturn melfa_hardware::MelfaHW::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    return CallbackReturn::SUCCESS;
    
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        melfa_hardware::MelfaHW, hardware_interface::SystemInterface)
