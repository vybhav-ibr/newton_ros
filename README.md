# Newton ROS Bridge (newton_ros) 🤖
![alt text](Newton-Tasks.gif)

`newton_ros` is a ROS 2 package designed to bridge the gap between the powerful [Newton](https://github.com/newton-physics/newton.git) simulator and the ROS 2 ecosystem. Newton is a next-gen physics platform for Robotics and Embodied AI, offering:

1.  A universal physics engine built from the ground up for a wide range of materials.
2.  A lightweight, ultra-fast, and user-friendly robotics simulation platform.
3.  A powerful and fast photo-realistic rendering system.
4.  A modular architecture which allows every part of the simulation pipeline from the render, the physisc solvers, teh contact pipeline ot be fully modular

This project provides the essential tools to kickstart your robotics development and simulation within ROS 2 using Newton.

---

## ✨ Key Features

`newton_ros` is a lightweight, pure Python package that simplifies the integration of Newton and ROS 2.

-   **ROS 2 Control Integration**: uses a topic based hardware interface for seamless compatibility with the `ros2_control` framework.
-   **Simulator Services**: Exposes core simulator functionalities (like pausing, resetting, domain_randomisation,inverse kinematics, forward kinematics, path planning) through ROS 2 services, defined in the accompanying `newton_ros_interfaces` package.
-   **Comprehensive Sensor Suite**: Provides a variety of simulated sensors, publishing data on standard ROS 2 topics.

---

## 🎮 Supported Sensors

`newton_ros` provides out-of-the-box support for several common robotics sensors:

-   📷 **Camera**: Publishes RGB, depth, semantic segmentation, and surface normal images.
-   📸 **RGBD Camera**: A convenient wrapper for synchronized RGB and point cloud publishing.
-   🛰️ **IMU**: Simulates an Inertial Measurement Unit.
-   📏 **Sectional Lidar**: A ray based lidar covering a section of the space(analogus to a depth camera).
-   🌐 **3D Lidar**: A ray based 360 degree 3D Lidar.
-   📡 **LaserScan** A 360 degree laser-scan with one vertical channel, commonly used in robotics 
-   💥 **Contact Force Sensor** A tactile sensor to measure the force experinced by a speified link
-   💥 **contact senor** A binary tactile sensor to check if an entity is in contact with any other entity 

---

## ⚙️ Installation

### Prerequisites

1.  **ROS 2**: This package is developed for ROS 2. Ensure you have a working installation (e.g., jazzy, kilted, rolling, humble).
2.  **Newton Simulator**: Install Newton by following the instructions in the official [Newton repository](https://github.com/newton-physics/newton.git).
    > **Note**: `newton_ros` was tested with Newton `v1.0.0`. Newer versions may have compatibility issues. Please report any problems you encounter!

### Steps

1.  **Downgrade NumPy(optional)**
    The default NumPy version installed with Newton may not be compatible with ROS 2. Downgrade to a compatible version:
    ```bash
    pip install numpy==1.26.4
    ```

2.  **Clone the Repository**
    Navigate to your Colcon workspace's `src` directory and clone this repository.
    ```bash
    # Example for a workspace in ~/ros2_ws
    cd ~/ros2_ws/src
    git clone https://github.com/vybhav-ibr/newton_ros.git .
    ```

3.  **Install Dependencies**
    Let `rosdep` handle the required dependencies.
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  **Build and Source the Workspace**
    Build the package using `colcon`.
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
    > **Troubleshooting**: If you encounter an `ImportError: No module named 'em'`, it might be due to a conflict with a Python virtual environment. Deactivate any active virtual environment run the `colcon clean workspace` command and try building with the `--merge-install` flag:
    > `colcon build --merge-install`

    After a successful build, source your workspace's setup file:
    ```bash
    source install/setup.bash
    ```

---

## 🧪 Example Usage

1. **Start the Newton Simulator and the newton_ros bridge**

    ```bash
    python test_import.py
    ```

2. **Launch the Ackermann Drive Demo**
    Run the provided demo from the `newton_ros2_control_demos` package to see a wheeled robot in action.

    ```bash
    ros2 launch newton_ros2_control_demos ackermann_drive_example.launch.py
    ```

---

## 📝 Known Issues & Limitations

-   **Newton Version**: newton is under active development. Using the latest version from their `main` branch may cause issues. Sticking to a tagged release like `v1.0.0` is recommended.
-   **Sensor Models**: Currently, no advanced sensor noise or distortion models are included. This is planned for a future release.
-   **topic_based_hardware_interfaces**: This external package [topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control.git) by picknik robotics is used for ros2_control support. The update frequncy for the topic-based-hardware-interface may be a limitation, This package is also not suitable for real-time controllers 

---

## 🤝 Contributing & Support

-   **Contributions Welcome!** Feel free to open a Pull Request to fix a bug or add a feature. If you'd like to contribute long-term, please open an issue to discuss it.
-   **Need Help?** For bugs, feature requests, or suggestions, please open an [issue](https://github.com/vybhav-ibr/Newton_ros/issues) in the repository.
-   **Configuration Docs**: For details on the parameters in the config files, please refer to the Newton source code, as the official documentation may not be up-to-date.
