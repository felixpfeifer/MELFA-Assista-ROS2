#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/interface_value.hpp"
#include "melfa_assista_hardware/visibility_control.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "controller_interface/controller_interface.hpp"

namespace melfa_assista_hardware
{
    // namespace melfa_assista_hardware
    using CmdType = std_msgs::msg::Float64MultiArray;
    class  GPIOController : public controller_interface::ControllerInterface{
    
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(GPIOController);

        melfa_assista_hardware_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        melfa_assista_hardware_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        melfa_assista_hardware_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        melfa_assista_hardware_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        melfa_assista_hardware_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        melfa_assista_hardware_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        melfa_assista_hardware_PUBLIC
        CallbackReturn on_init() override;

        private:
        std::vector<std::string> inputs_;
        std::vector<std::string> outputs_;

        protected:
        void initMsgs();

        // internal commands
        std::shared_ptr<CmdType> output_cmd_ptr_;

        // publisher
        std::shared_ptr<rclcpp::Publisher<control_msgs::msg::InterfaceValue>> gpio_publisher_;
        control_msgs::msg::InterfaceValue gpio_msg_;

        // subscriber
        rclcpp::Subscription<CmdType>::SharedPtr subscription_command_;
    };
        
}