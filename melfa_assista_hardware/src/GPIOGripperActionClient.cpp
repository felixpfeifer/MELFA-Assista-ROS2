//
// Created by felix on 24.05.24.
//

#include "melfa_assista_hardware/GPIOGripperActionClient.hpp"

namespace melfa_assista_hardware {
    using std::placeholders::_1;

    GPIOGripperActionClient::GPIOGripperActionClient() : Node("GPIOGripperActionClient") {
        this->client_ptr_ = rclcpp_action::create_client<GripperCMD>(this, "/gripper_action_controller/gripper_cmd");
        subscription_ = this->create_subscription<CmdType>("/gpio_controller/inputs", rclcpp::SystemDefaultsQoS(),
                                                           std::bind(&GPIOGripperActionClient::callback_states,
                                                                     this, _1));
        RCLCPP_INFO(this->get_logger(),"GPIOGripperActionClient is starting");
    }

    void GPIOGripperActionClient::send_goal(bool isClosed) {
        using namespace std::placeholders;
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
        auto goal_msg = GripperCMD::Goal();
        goal_msg.command.max_effort = maxeffort_;
        goal_msg.command.position = isClosed ? 0.0 : 0.006; // 6mm MAX Open

        auto send_goal_options = rclcpp_action::Client<GripperCMD>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&GPIOGripperActionClient::result_callback, this, _1);
        send_goal_options.goal_response_callback = std::bind(&GPIOGripperActionClient::goal_response_callback, this,
                                                             _1);
        send_goal_options.feedback_callback = std::bind(&GPIOGripperActionClient::feedback_callback, this, _1, _2);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(),"Update State of the Gripper");

    }

    void GPIOGripperActionClient::goal_response_callback(const GoalHandelGripper::SharedPtr &goalHandle) {
        if (!goalHandle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void GPIOGripperActionClient::feedback_callback(GoalHandelGripper::SharedPtr,
                                                    const std::shared_ptr<const GripperCMD::Feedback> feedback) {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        ss << feedback->position;
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    }

    void GPIOGripperActionClient::result_callback(const GoalHandelGripper::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        ss << result.result->position;
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        //rclcpp::shutdown();
    }

    void GPIOGripperActionClient::callback_states(const CmdType &msg) {
        if (msg.values.size() != 1 || msg.values.empty()) {
            return;
        } else {
            bool state = false;
            if (msg.values[0] == closed) {
                state = true;
            }

            bool isChanged = isClosed_ != state;
            if(!isChanged){
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Send to Action Server with goal %d", state);
            this->send_goal(state);
            isClosed_ = state;
            RCLCPP_INFO(this->get_logger(),"Gripper is changed");

        }

    }
}

    int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<melfa_assista_hardware::GPIOGripperActionClient>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }

