/**
 * Author Felix Pfeifer
 * Version 1.0
 */

#ifndef MELFA_HARDWARE_INTERFACE_HPP_
#define MELFA_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <map>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <rclcpp/rclcpp.hpp>

#include "melfa_assita_ros2_hw/strdef.hpp"
#include "melfa_assita_ros2_hw/visibility_control.h"

#define joint_number 6
#define _period 0.0071
#define PORT 10000

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * Hardware Interface for a 6 Axes MELFA Robot
 * 
 */
namespace melfa_hardware
{
class MelfaHW : public hardware_interface::SystemInterface {
public:

    /**
     * Constructor of the Hardware Interface
     * Loads the urdf and gets the roboter ip Adress
     *
    */
    MELFA_HARDWARE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    /**
     * Exports the position state interfaces of the robot
     * @return Vector of the state interfaces
    */
    MELFA_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
    * Exports the position command interfaces of the robot
    * @return Vector of the command interfaces
    */
    MELFA_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MELFA_HARDWARE_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;


    MELFA_HARDWARE_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;



    /**
     * Reads the network packet of the MELFA-robot and returns the position data to the state interface
     * @param time
     * @param period
     * @return OK, if the robot sends data
     */
    MELFA_HARDWARE_PUBLIC
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /**
     * Configures the hardware interface
     * Inits the communication with the robot and gets the current position of the robot
    */
    MELFA_HARDWARE_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    /**
     * Cleans up the hardware interface
     * Closes the connection to the robot
     */
    MELFA_HARDWARE_PUBLIC
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    

    /**
     * Sends a network packet over ethernet to the robot
     * @return SUCCESS if packet got send ERROR otherwise
     */
    MELFA_HARDWARE_PUBLIC
    return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    /// The size of this vector is (standard_interfaces_.size() x nr_joints)
    std::vector<double> joint_position_command_; // Vector for the joint position command
    std::vector<double> joint_position_state_; // Vector for the joint position state
private:
    MXTCMD send_buff; // Buffer for the Joint Position Command with the MXT-Command Packet
    MXTCMD recv_buff; // Buffer for the Joint Position Position with the MXT-Command Packet
    std::string robot_ip; // IP-Address of the robot as a string
    struct sockaddr_in addr; // Struct for the socket address
    int counter_;
    int sock;  // Socket for the connection to the robot
    rclcpp::Logger logger = rclcpp::get_logger("melfa_assita_ros2_hw"); // Logger for the hardware interface 

};
}

#endif  // MELFA_HARDWARE_INTERFACE_HPP_