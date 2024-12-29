# ROS2 Package for the MELFA ASSISTA

This ROS2 package provides an interface to control the Mitsubishi Electric MELFA ASSISTA cobot. It includes drivers and utilities for integrating the cobot with the Schunk Coact Assista Gripper and preparing for Sensopart V20 Camera integration. The package is designed to work with ROS2 Humble and supports basic motion control and gripper operations, with the camera included in the URDF model and mock robot launch.

---

## Features

- **Cobot Control**: ROS2 nodes for controlling the MELFA ASSISTA robotic arm.
- **Gripper Integration**: Interface for operating the Schunk Coact Assista Gripper.
- **GPIO-Based Gripper Control**: A ROS2 action client (`GPIOGripperActionClient`) for controlling the gripper using GPIO inputs.
- **Camera URDF Model**: Sensopart V20 Camera included in the URDF for visualization and future integration.
- **Modular Design**: Easy to extend and customize for specific use cases.
- **ROS2 Standards**: Adheres to ROS2 Humble conventions and practices.

---

## Parts Used in the Package

### Hardware Components

1. **[Mitsubishi Electric – MELFA ASSISTA](https://de.mitsubishielectric.com/fa/products/rbt/assista)**:

   - A collaborative robotic arm designed for precise and efficient operations in industrial and research settings.

2. **[Schunk Coact Assista Gripper](https://schunk.com/de/de/greiftechnik/parallelgreifer/co-act-egp-c/co-act-egp-c-40-n-n-assista/p/000000000001408586)**:

   - A versatile and safe gripper optimized for collaborative robot applications.

3. **[Sensopart V20 Camera](https://www.sensopart.com)**:

   - An industrial camera included in the URDF model for visualization and future tasks.

---

## Installation

1. **Clone the Repository**:

   ```bash
   git clone https://github.com/your-username/melfa-assista-ros2.git
   cd melfa-assista-ros2
   ```

2. **Install Dependencies**:
   Use `rosdep` to install all required dependencies:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Package**:

   ```bash
   colcon build
   source install/setup.bash
   ```

---

## Usage

1. **Launch the Mock Robot**:
   Start the mock robot interface for simulation:

   ```bash
   ros2 launch melfa_assista_driver launch_mock_robot.launch.py
   ```

2. **Launch the Real Robot**:
   Start the MELFA ASSISTA driver node for the real robot. Specify the robot IP:

   ```bash
   ros2 launch melfa_assista_driver melfa_controller.launch.py robot_ip:=<your_robot_ip>
   ```

3. **Launch the Movegroup**:
   To use MoveIt for the robot launch the movegroup with
   ```bash
   ros2 launch melfa_assista_moveit_config move_group.launch.py 
   ```
---

## Configuration

- **Parameter Files**: Located in the `config/` directory for customizing cobot and gripper settings.
- **URDF Model**: A complete description of the MELFA ASSISTA and attached peripherals, including the Sensopart V20 Camera, can be found in `urdf/`.
- **Real Robot IP Configuration**: Set the `robot_ip` argument in `melfa_controller.launch.py` to connect to the actual robot.
- **GPIO Gripper Control**: Ensure the `/gpio_controller/inputs` topic is published with valid input states for the `GPIOGripperActionClient` to function correctly.

---

## References

- [Gripper](https://schunk.com/de/de/greiftechnik/parallelgreifer/co-act-egp-c/co-act-egp-c-40-n-n-assista/p/000000000001408586)
- [Structure and Tutorials](https://articulatedrobotics.xyz/)

---

## Contribution

Contributions are welcome! Please follow the [ROS2 contribution guidelines](https://index.ros.org/doc/ros2/Contributing/) when submitting issues or pull requests.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Contact

For questions or support, please reach out to [Felix Pfeifer](fpfeifer@stud.hs-heilbronn.de).

