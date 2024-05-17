//
// Created by felix on 05.03.24.
//

#include "melfa_assista_hardware/melfa_assista_hardware_interface.hpp"

CallbackReturn melfa_assista_hardware::MelfaHW::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
        return CallbackReturn::ERROR;
    }
    
    _robot_ip = info_.hardware_parameters["robot_ip"];

    // Set the to Vectors with 0 Values for each of the six Joints
    joint_position_state_.resize(6,0.0);
    joint_position_command_.resize(6,0.0);
    //gpio_state_.resize(1,0);
    //gpio_command_.resize(1,0);
    

    for(const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        // MELFA Supports only a Position Interface and State for each Joint
        if(joint.command_interfaces.size() != 1){
            RCLCPP_FATAL(_logger,"Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(
            _logger,
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
        RCLCPP_FATAL(
            _logger,
            "Joint '%s' has %d state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(
            _logger,
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
        }
    }
    //return CallbackReturn::SUCCESS;
    // GPIO Initialization
    if (info_.gpios.size() != 1)
    {
        RCLCPP_FATAL(_logger, "1 GPIOs expected, %d found", info_.gpios.size());
        return CallbackReturn::ERROR;
    }
    
    // With one Command and one State Interface
    if (info_.gpios[0].command_interfaces.size() != 1)
    {
        RCLCPP_FATAL(
            _logger,
            "GPIO '%s' has %d command interfaces found. 1 expected.", info_.gpios[0].name.c_str(),
            info_.gpios[0].command_interfaces.size());
        return CallbackReturn::ERROR;
    }
    if (info_.gpios[0].state_interfaces.size() != 1)
    {
        RCLCPP_FATAL(
            _logger,
            "GPIO '%s' has %d state interfaces found. 1 expected.", info_.gpios[0].name.c_str(),
            info_.gpios[0].state_interfaces.size());
        return CallbackReturn::ERROR;
    }


    return CallbackReturn::SUCCESS;
    }


std::vector<hardware_interface::StateInterface> melfa_assista_hardware::MelfaHW::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_state_[i]));
        RCLCPP_INFO(_logger,"Joint %s", info_.joints[i].name.c_str());
    }

    size_t ct = 0;
    gpio_state_.resize(info_.gpios.size());
    for (size_t i = 0; i < info_.gpios.size(); i++)
    {
        for (auto state_if : info_.gpios.at(i).state_interfaces)
        {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios.at(i).name, state_if.name, &gpio_state_[ct++]));
        RCLCPP_INFO(
            rclcpp::get_logger("GPIOController"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), state_if.name.c_str());
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> melfa_assista_hardware::MelfaHW::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
    }
    gpio_command_.resize(info_.gpios.size());
    size_t ct = 0;
    for (uint i = 0; i < info_.gpios.size(); i++)
    {
         for (auto command_if : info_.gpios.at(i).command_interfaces)
         {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios.at(i).name, command_if.name, &gpio_command_[ct++]));
            RCLCPP_INFO(
                rclcpp::get_logger("GPIOController"), "Added %s/%s",
                info_.gpios.at(i).name.c_str(), command_if.name.c_str());
        }
    }

    return command_interfaces;

}

/**
 * @brief Function to write the command Position to the MELFA Robot
*/
return_type melfa_assista_hardware::MelfaHW::write(const rclcpp::Time &, const rclcpp::Duration &) {
    //return return_type::OK;
    _counter++;

    memset(&_send_buff,0,sizeof(_send_buff));
    _send_buff.Command = MXT_CMD_MOVE;
    _send_buff.SendType = MXT_TYP_JOINT;
    _send_buff.RecvType = MXT_TYP_JOINT;

    _send_buff.SendIOType = MXT_IO_OUT;
    _send_buff.RecvIOType = MXT_IO_IN;
    _send_buff.BitTop = STARTINGBIT; // First bit of the 16 Bits array
    _send_buff.BitMask = ((1 << 1) | (1 << 0)); // Mask for the Databits of the 16 Bits, used are the first two
    
    if (gpio_command_[0] > 0.90) // Open
    {
        _send_buff.IoData = (0 << CLOSE | 1 << OPEN);
    }
    else // Close
    {
        _send_buff.IoData = (1 << CLOSE | 0 << OPEN);
    }
    _send_buff.CCount = _counter;

    _send_buff.dat.jnt.j1 = (float)joint_position_command_[0];
    _send_buff.dat.jnt.j2 = (float)joint_position_command_[1];
    _send_buff.dat.jnt.j3 = (float)joint_position_command_[2];
    _send_buff.dat.jnt.j4 = (float)joint_position_command_[3];
    _send_buff.dat.jnt.j5 = (float)joint_position_command_[4];
    _send_buff.dat.jnt.j6 = (float)joint_position_command_[5];

    int size = sendto (_socket, (char *) &_send_buff, sizeof (_send_buff), 0, (struct sockaddr *) &_addres, sizeof (_addres));
    //TODO: Try to send the packet to the robot for 1000 trys then return ERROR

    if (size != sizeof (_send_buff))
    {
       std::cout << "Can't send to Controller" << "\n";

    }

    return return_type::OK;
 
    
}

return_type melfa_assista_hardware::MelfaHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {

    int n = recvfrom(_socket,&_recv_buff,sizeof(_recv_buff),0,NULL,NULL);
    if (n < 0)
    {
        std::cout << "Cannot get packed to Controller" << "\n";
    }
    JOINT *joints = (JOINT *) &_recv_buff.dat;
    joint_position_state_[0] = joints->j1;
    joint_position_state_[1] = joints->j2;
    joint_position_state_[2] = joints->j3;
    joint_position_state_[3] = joints->j4;
    joint_position_state_[4] = joints->j5;
    joint_position_state_[5] = joints->j6;
    //std::cout << "Succesful get Postion  of the Robot" << "\n";
    // Read Out the State of the Gripper
    if (_recv_buff.IoData & (1 << OPEN))
    {
        gpio_state_[0] = 1.0;
    }
    else
    {
        gpio_state_[0] = 0.0;
    }

    return return_type::OK;

}

/**
*/
CallbackReturn melfa_assista_hardware::MelfaHW::on_configure(const rclcpp_lifecycle::State &previous_state) {
    return CallbackReturn::SUCCESS;
}

CallbackReturn melfa_assista_hardware::MelfaHW::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    return CallbackReturn::SUCCESS;

}


CallbackReturn melfa_assista_hardware::MelfaHW::on_activate(const rclcpp_lifecycle::State &previous_state) {


    // TODO: Implement to check if the robot is connected
    // TODO: Implement the R3 Automation Protocol to start the Robot remotely

    //return CallbackReturn::SUCCESS;
    _counter = 0;
    // Create the Socket for the Robot
    _socket = socket(AF_INET,SOCK_DGRAM,0);

    // Get the inital state from the robot
    fd_set fds;
    timeval _time;

    FD_ZERO (&fds);
    FD_SET (_socket,&fds);

    _time.tv_sec = 0;
    _time.tv_usec = 100000;

    if(_socket < 0){
        std::cout << "Can't create a Socket" << "\n";
        //return CallbackReturn::FAILURE;
    }

    if (int err = setsockopt(_socket, SOL_SOCKET, SO_RCVTIMEO,(char*)&_time,sizeof(_time))) {
        std::cout << "Cant set socket options" << "\n";
        return CallbackReturn::FAILURE;
    }

    memset(&_addres,0,sizeof(_addres));
    _addres.sin_port = htons(10000);
    _addres.sin_family = AF_INET;
    _addres.sin_addr.s_addr = inet_addr(_robot_ip.c_str());
    if (_addres.sin_addr.s_addr == INADDR_NONE) {
        std::cout << "IP Addres is not valid" << "\n";
        return CallbackReturn::FAILURE;
    }

    // Packet to Send send_buffer
    memset(&_send_buff,0,sizeof(_send_buff));

    
    _send_buff.Command = MXT_CMD_NULL;
    _send_buff.SendType = MXT_TYP_NULL;
    _send_buff.RecvType = MXT_TYP_JOINT;
    _send_buff.SendIOType = MXT_IO_OUT;
    _send_buff.RecvIOType = MXT_IO_IN;
    _send_buff.BitTop = STARTINGBIT; // First bit of the 16 Bits array
    _send_buff.BitMask = ((1 << 1) | (1 << 0)); // Mask for the Databits of the 16 Bits, used are the first two
    _send_buff.IoData = (0 << CLOSE | 1 << OPEN);

    _send_buff.CCount = _counter;

    // Write GPIO Vectors
    gpio_command_[0] = 1.0;
    gpio_state_[0] = 1.0;

    auto size = sendto(_socket,&_send_buff,sizeof(_send_buff),NULL,(const struct sockaddr *) &_addres,sizeof(_addres));
    if (size != sizeof(_send_buff)){
        std::cout << "Cannot send packed to Controller" << "\n";
    }

    std::cout << "Succesful send emtpy start Command Robot" << "\n";
    memset(&_recv_buff,0,sizeof(_recv_buff));



    int status = select(_socket, &fds, (fd_set *) NULL, (fd_set *) NULL, &_time);
    if (status < 0)
    {
        std::cout <<  "Can't reciv Package" << "\n";
    }

    int n = recvfrom(_socket,&_recv_buff,sizeof(_recv_buff),0,NULL,NULL);
    if (n < 0)
    {
        std::cout << "Cannot get packed to Controller" << "\n";
    }
    JOINT *joints = (JOINT *) &_recv_buff.dat;
    joint_position_state_[0] = joints->j1;
    joint_position_state_[1] = joints->j2;
    joint_position_state_[2] = joints->j3;
    joint_position_state_[3] = joints->j4;
    joint_position_state_[4] = joints->j5;
    joint_position_state_[5] = joints->j6;
    //std::cout << "Succesful get Postion to inital of the Robot" << "\n";

    joint_position_command_  =  joint_position_state_;
    //std::cout << "Succesful set Postion to inital of the Robot" << "\n";
}

CallbackReturn melfa_assista_hardware::MelfaHW::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    return CallbackReturn::SUCCESS;
    
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        melfa_assista_hardware::MelfaHW, hardware_interface::SystemInterface)
