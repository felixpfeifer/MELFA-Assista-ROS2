/**
 * Author Felix Pfeifer
 * Version 1.0
 */

#ifndef melfa_assista_hardware_INTERFACE_HPP_
#define melfa_assista_hardware_INTERFACE_HPP_

#include <string>
#include <vector>
#include <map>
#include <algorithm> // Für std::remove
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <rclcpp/rclcpp.hpp>

#include "melfa_assista_hardware/strdef.hpp"
#include "melfa_assista_hardware/visibility_control.h"

#define joint_number 6
#define _period 0.0071
#define PORT 10000

// Typedefs for the Tool/Gripper
#define STARTINGBIT 900

#define OPEN 0
#define CLOSE 1

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * Hardware Interface for a 6 Axes MELFA Robot
 *
 */
namespace melfa_assista_hardware {
    class MelfaHW : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MelfaHW);

        /**
         * Constructor of the Hardware Interface
         * Loads the urdf and gets the roboter ip Adress
         *
         */
        melfa_assista_hardware_PUBLIC
        CallbackReturn
        on_init(const hardware_interface::HardwareInfo &info) override;

        /**
         * Exports the position state interfaces of the robot
         * @return Vector of the state interfaces
         */
        melfa_assista_hardware_PUBLIC
        std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;

        /**
         * Exports the position command interfaces of the robot
         * @return Vector of the command interfaces
         */
        melfa_assista_hardware_PUBLIC
        std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

        melfa_assista_hardware_PUBLIC
        CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;

        melfa_assista_hardware_PUBLIC
        CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /**
         * Reads the network packet of the MELFA-robot and returns the position data to the state interface
         * @param time
         * @param period
         * @return OK, if the robot sends data
         */
        melfa_assista_hardware_PUBLIC
        return_type
        read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /**
         * Configures the hardware interface
         * Inits the communication with the robot and gets the current position of the robot
         */
        melfa_assista_hardware_PUBLIC
        CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;

        /**
         * Cleans up the hardware interface
         * Closes the connection to the robot
         */
        melfa_assista_hardware_PUBLIC
        CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        /**
         * Sends a network packet over ethernet to the robot
         * @return SUCCESS if packet got send ERROR otherwise
         */
        melfa_assista_hardware_PUBLIC
        return_type
        write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    protected:
        /// The size of this vector is (standard_interfaces_.size() x nr_joints)
        std::vector<double> joint_position_command_; // Vector for the joint position command
        std::vector<double> joint_position_state_;   // Vector for the joint position state

        std::vector<double> tool_position_command_; // Variable for the tool position command
        std::vector<double> tool_velocity_state_;   // Variable for the tool velocity state
        std::vector<double> tool_position_state_;   // Variable for the tool position state

        // Vectors for the Joints
        std::vector<double> gpio_state_;   // Vector for the gpio state
        std::vector<double> gpio_command_; // Vector for the gpio command

    private:
        MXTCMD _send_buff;          // Buffer for the Joint Position Command with the MXT-Command Packet
        MXTCMD _recv_buff;          // Buffer for the Joint Position Position with the MXT-Command Packet
        std::string _robot_ip;      // IP-Address of the robot as a string
        struct sockaddr_in _addres; // Struct for the socket address
        int _counter;
        int _socket;                                                           // Socket for the connection to the robot
        rclcpp::Logger _logger = rclcpp::get_logger("melfa_assista_hardware"); // Logger for the hardware interface
        std::string tool_joint_name = "joint_hand_left";
        std::string tool_joint_name2 = "joint_hand_right";
    };
}

#endif // melfa_assista_hardware_INTERFACE_HPP_