# ros2-room3d-mapper-using-2d-lidar
ROS 2-based 3D indoor mapping system using 2D LiDAR, odometry, dynamic LiDAR tilt, point-cloud reconstruction, and RViz visualization.
# Real-Time 3D Room Reconstruction with ROS 2 and Tilting LiDAR

This project is a real-time 3D space reconstruction system using a 2D LiDAR sensor (RPLiDAR A1M8) driven by a servo motor for vertical sweeping. The system utilizes a distributed architecture between a Raspberry Pi (running on the robot) and a PC/Laptop (running VMware) using ROS 2 Humble.

##  Hardware Specifications

*   **Computation:** Raspberry Pi 4 Model B (Robot) & PC/Laptop with VMware (Visualization & Control)
*   **Microcontroller:** Arduino Mega 2560 + ESP8266 Module (For data communication via Wi-Fi)
*   **Sensor:** RPLIDAR A1M8
*   **Actuators:**
    *   1x TD8120MG Servo (LiDAR tilt driver)
    *   2x YGY6138-R528C-104-1265E DC Motors with Encoders
*   **Motor Driver:** L298N
*   **Power Source (LiPo Batteries & LM2596 Step-Down):**
    *   2x LiPo Batteries (3.7V) -> 5V Step-down -> Raspberry Pi
    *   2x LiPo Batteries (3.7V) -> 5V Step-down -> Arduino Mega 2560
    *   4x LiPo Batteries (3.7V) -> 12V Step-down -> L298N Motor Driver

---

##  Step 1: Hardware & Wiring Setup

1.  **Power Distribution:** Ensure the LM2596 step-down modules are calibrated to the correct voltages (5V and 12V) using a multimeter **before** connecting them to the components (Raspberry Pi, Arduino, or L298N) to prevent damage from over-voltage.
2.  **Motor & Encoder Wiring:** 
    *   Connect the DC motors to the L298N outputs.
    *   Connect the L298N control pins to the Arduino Mega: Left Motor (PWM 9, IN1 7, IN2 8) and Right Motor (PWM 10, IN1 11, IN2 12).
    *   Connect the Encoder pins to the Arduino Mega: Left (Pin 2, 4) and Right (Pin 3, 5).
3.  **Servo Wiring:** Connect the TD8120MG Servo signal pin to Pin 6 on the Arduino.
4.  **Communication Connection:** Since the serial data output is transmitted via Wi-Fi, ensure the Arduino Mega 2560 is connected to the ESP8266 module. Connect the Arduino TX/RX pins to the ESP8266 RX/TX pins (via a logic level converter if necessary).

---

##  Step 2: Arduino & ESP8266 Firmware Setup

1.  Open the Arduino IDE.
2.  Open the `robot_base.ino` (or `robot_base_2.ino`) file.
3.  Upload the code to the Arduino Mega 2560.
4.  Configure the ESP8266 module to operate as a transparent TCP Server. This module is responsible for forwarding string commands from the Wi-Fi connection to the Arduino's serial port (Baudrate `115200`), and returning the response strings (e.g., `E <tickL> <tickR> <tilt>`) back to the client (Raspberry Pi).

> **Network Communication Note:**
> On the Raspberry Pi side, since the `odom_node.py` file reads input through a serial port like `/dev/ttyUSB1`, you need to create a Virtual Serial Port that bridges the Wi-Fi connection from the ESP8266. You can use the `socat` utility in the Raspberry Pi terminal:
> ```bash
> sudo apt install socat
> socat pty,link=/dev/ttyWIFI,waitslave tcp:<ESP8266_IP>:<ESP8266_PORT>
> ```
> *(Use `/dev/ttyWIFI` as the `base_port` parameter when running `pi_3d.launch.py`)*

---

##  Step 3: ROS 2 Workspace Installation (Raspberry Pi & Laptop)

Perform these steps on **both devices** (Ubuntu 22.04 on Raspberry Pi and VMware Laptop):

1.  **Environment Preparation:** Ensure ROS 2 Humble is installed.
2.  **Create Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
3.  **Add Source Code:** Create a package folder named `room3d` and place all the code files (`cloud_mapper.py`, `odom_node.py`, `arrow_teleop.py`, `package.xml`, `setup.py`, launch files, rviz, and urdf) according to the structure defined in `setup.py`.
4.  **Install Dependencies:**
    ```bash
    sudo apt update
    sudo apt install python3-serial ros-humble-sllidar-ros2 ros-humble-robot-state-publisher ros-humble-rviz2
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
5.  **Build the Package:**
    ```bash
    colcon build --packages-select room3d
    ```
6.  **Source the Workspace:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
    *(Add the above command to your `~/.bashrc` to run it automatically every time you open a terminal).*

---

##  Step 4: How to Run the System

This system is designed so that the LiDAR uploads the point cloud HANYA (ONLY) when the robot is stopped (stop-and-scan). Ensure the Raspberry Pi and Laptop (VMware) are on the same Wi-Fi network and the `ROS_DOMAIN_ID` variable is configured identically on both devices.

### On the Raspberry Pi (Robot)
Run the `socat` Wi-Fi bridge (if using the virtual port), then launch the sensor and odometry:
```bash
ros2 launch room3d pi_3d.launch.py lidar_port:=/dev/ttyUSB0 base_port:=/dev/ttyWIFI
```

### On the Laptop / VMware (Control & Visualization)
Open two separate terminal tabs.

**Terminal 1 (Visualization & 3D Mapping):**
Run the state publisher, cloud mapper, and RViz.
```bash
ros2 launch room3d viz_3d.launch.py
```

**Terminal 2 (Teleoperation):**
Use this node to control the robot's movement via the keyboard (W/A/S/D or Arrow keys, and Space to stop).
```bash
ros2 run room3d arrow_teleop
```

---

##  Keyboard Control Guide (arrow_teleop)
*   `A` / `Up Arrow`: Move Forward
*   `B` / `Down Arrow`: Move Backward
*   `D` / `Left Arrow`: Turn Left (CCW)
*   `C` / `Right Arrow`: Turn Right (CW)
*   `Space`: Full Stop (The robot must be stopped for the cloud mapper to record 3D data)
*   `T`: Toggle servo sweeping ON/OFF manually
*   `C`: Clear the current point cloud data
*   `V`: Save the point cloud to a `.pcd` file (Saved at `~/room3d_scan.pcd`)
