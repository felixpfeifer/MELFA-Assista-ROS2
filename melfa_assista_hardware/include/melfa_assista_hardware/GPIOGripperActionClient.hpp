// Created by felix on 24.05.24.
//

#ifndef MELFA_ASSISTA_HARDWARE_GPIOGRIPPERACTIONCLIENT_HPP
#define MELFA_ASSISTA_HARDWARE_GPIOGRIPPERACTIONCLIENT_HPP

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <control_msgs/action/gripper_command.hpp>
#include "control_msgs/msg/interface_value.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/rclcpp_components/register_node_macro.hpp>

namespace melfa_assista_hardware {
    using GripperCMD = control_msgs::action::GripperCommand;
    using GoalHandelGripper = rclcpp_action::ClientGoalHandle<GripperCMD>;

    using CmdType = control_msgs::msg::InterfaceValue;

    class GPIOGripperActionClient : public rclcpp::Node {
    public:
        explicit GPIOGripperActionClient();

        void send_goal(bool isClosed);

        void callback_states(const CmdType &cmd);

    private:

        rclcpp_action::Client<GripperCMD>::SharedPtr client_ptr_;

        void goal_response_callback(const GoalHandelGripper::SharedPtr & goalHandle);

        void feedback_callback( GoalHandelGripper::SharedPtr,
                                const std::shared_ptr<const GripperCMD::Feedback> feedback) ;

        void result_callback(const GoalHandelGripper::WrappedResult & result);



        rclcpp::Subscription<CmdType>::SharedPtr subscription_;

        bool isClosed_ = false;
        const double maxeffort_ = 160.0;
        double closed = 1.0;

        const std::string topic_states_name_ = "~/inputs";

    };
}


#endif //MELFA_ASSISTA_HARDWARE_GPIOGRIPPERACTIONCLIENT_HPP
