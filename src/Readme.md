# 🤖 ROS2 Mobile Robot Masterclass: Master Workshop Manual

**Author & Maintainer:** [Rahul Ramasamy](https://github.com/rahul-r-joshua)  
* 🐙 **GitHub:** [https://github.com/rahul-r-joshua](https://github.com/rahul-r-joshua)  
* 🌐 **Portfolio:** [https://rahul-r-joshua.github.io/portfolio/](https://rahul-r-joshua.github.io/portfolio/)  
* 💼 **LinkedIn:** [https://www.linkedin.com/in/rahul-ramasamy-in/](https://www.linkedin.com/in/rahul-ramasamy-in/)  
* 📧 **Email:** rahul.r.joshua123@gmail.com  

---

## 📖 About This Workshop

Welcome to the **ROS 2 Mobile Robotics Masterclass Workshop Manual**. This comprehensive engineering guide provides complete step-by-step documentation covering mechanical modelling, 3-engine physics simulation, kinematics formulation, priority teleoperation, active LiDAR collision filtering, serial hardware bridging, and physical deployment on Raspberry Pi 4 / 5 with Arduino and ESP32.

### 🗺️ Quick Navigation Curriculum Index

| Milestone / Section | Package / Focus | Direct Jump |
| :--- | :--- | :---: |
| **Milestone 1: 3D URDF & Robot Modeling** | `mobile_robot_description` | [Jump to Milestone 1](#milestone-1) |
| **Milestone 2: Differential Drive Kinematics & Odometry** | `mobile_robot_controller` | [Jump to Milestone 2](#milestone-2) |
| **Milestone 3: Teleoperation, Twist Mux & Joystick Failsafe** | `mobile_robot_controller` | [Jump to Milestone 3](#milestone-3) |
| **Milestone 4: Active LiDAR Safety Zones (C++ & Python)** | `mobile_robot_bringup` | [Jump to Milestone 4](#milestone-4) |
| **Milestone 5: Master Simulation Bringup Launch** | `mobile_robot_bringup` | [Jump to Milestone 5](#milestone-5) |
| **🛠️ Track 1: Simulation Troubleshooting Guide** | Gazebo & ROS 2 Control Diagnostics | [Jump to Sim Troubleshooting](#troubleshooting-simulation) |
| **Milestone 6: Microcontroller Firmware & Serial Bridge** | `mobile_robot_firmware` | [Jump to Milestone 6](#milestone-6) |
| **📦 Physical Robot Bill of Materials (BOM)** | Component Specifications & Part List | [Jump to Bill of Materials](#bill-of-materials) |
| **🔌 4 Complete Hardware Wiring Schematics** | Arduino & ESP32 Circuit Diagrams | [Jump to Wiring Diagrams](#wiring-diagrams) |
| **Milestone 7: Real Robot Hardware Bringup** | `mobile_robot_bringup` | [Jump to Milestone 7](#milestone-7) |
| **🍓 Raspberry Pi 4 / 5 Onboard OS & Hardware Setup** | Ubuntu 22.04 LTS, UART & udev Rules | [Jump to Raspberry Pi Setup](#raspberry-pi-setup) |
| **🔌 Track 2: Embedded Hardware & Serial Troubleshooting** | 16-Point Hardware Matrix & Diagnostics | [Jump to HW Troubleshooting](#troubleshooting-hardware) |

---

# 💻 TRACK 1: SIMULATION & CONTROL ARCHITECTURE (Simulation Track)

> [!NOTE]
> Milestones 1 through 5 provide the complete, end-to-end robotics workshop entirely in high-fidelity simulation (Gazebo Classic, Ignition, or Modern Gz + RViz) without requiring physical hardware.

---

## ⚙️ Step 0: Prerequisites, Dependencies & Workspace Setup

Before beginning the workshop, install the necessary ROS 2 Humble packages, dependencies, and initialize the ROS 2 workspace:

### 0.1 Install Core ROS 2 Dependencies, gedit & Build Tools

Run the automated installer script:
```bash
bash scripts/install_ros2_dependencies.sh
```

Or install all packages manually via `apt`:
```bash
sudo apt update && sudo apt install -y \
  gedit \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pytest \
  python3-serial \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-interfaces \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-twist-mux \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools
```

### 0.2 Configure Hardware Permissions & Environment Sourcing
Add your Linux user to the `dialout` and `input` groups for USB serial and joystick controller access:
```bash
sudo usermod -a -G dialout,input $USER
```
*(Note: Log out and log back in, or run `newgrp dialout`, for group permissions to take effect).*

Ensure the ROS 2 Humble environment is sourced automatically in your shell:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 0.3 Initialize Workspace Structure
Create the workspace root and source directory:
```bash
mkdir -p ~/ros2_mobile_robot_ws/src
```

> [!TIP]
> ### 💡 Multi-Terminal Sourcing & Headless / CLI Editor Guide
> 1. **Multi-Terminal Sourcing:** In **EVERY** new terminal tab or window you open throughout this guide, ensure your ROS 2 workspace environment is loaded:
>    ```bash
>    source /opt/ros/humble/setup.bash
>    source ~/ros2_mobile_robot_ws/install/setup.bash
>    ```
> 2. **Editing Files in Non-GUI / SSH / Docker Environments:** If you are working on a headless Linux server, SSH remote session, Docker container, or WSL without X11 where `gedit` is unavailable, you can edit files using `nano`, `vim`, `code`, or write files directly from the terminal using bash heredocs:
>    ```bash
>    cat << 'EOF' > path/to/file
>    # [Paste file content here]
>    EOF
>    ```

---

<a id="milestone-1" name="milestone-1"></a>
## Milestone 1
### 3D Robot Modeling, URDF & Physics Simulation (`mobile_robot_description`)

In Milestone 1, we focus purely on the mechanical, kinematic, and visual modeling of our 4-wheel mobile robot:
1. **Geometric Links & Coordinate Frames:** `base_footprint`, `base_link`, 4 continuous wheel joints, and sensor mount frames (`lidar_link`, `camera_link`, `imu_link`).
2. **Inertia & Mass Properties:** Accurate cylinder and cuboid moments of inertia to ensure physics stability.
3. **Contact Dynamics & Skid-Steer Friction:** Tuned longitudinal rolling traction (`mu1: 0.8`) and lateral slip (`mu2: 0.1`) to eliminate wheel jitter.
4. **Sensor Simulation:** 2D LiDAR (`/scan`), Front Camera (`/camera/image_raw`), and 9-axis IMU (`/imu/data`).
5. **Visual Inspection:** Testing mechanical articulation with RViz joint sliders (`display.launch.py gui:=true`).
6. **Physics Spawn:** Spawning the robot into an empty physics world in your choice of simulator (Gazebo Classic, Ignition, or Modern Gz).

> [!IMPORTANT]
> **Pedagogical Boundary:** In Milestone 1, the robot model defines **strictly** the physical body, collision bounds, inertial tensors, and sensor plugins. There are **no motor controllers** in Milestone 1. Actuation interfaces (`ros2_control.xacro`, `transmission.xacro`, and controller plugins) are introduced in Milestone 2 when we transition to active motor control.

### Step 1.1: Create the Package with `ros2 pkg create`
```bash
cd ~/ros2_mobile_robot_ws/src
ros2 pkg create --build-type ament_cmake mobile_robot_description
```

### Step 1.2: Create Subdirectories
```bash
cd ~/ros2_mobile_robot_ws/src/mobile_robot_description
mkdir -p urdf launch rviz
```

### Step 1.3: Create Master URDF (`urdf/mobile_robot.urdf.xacro`)
The master URDF defines the robot's physical links and joints:
* `base_footprint`: Ground-plane projection frame ($z = 0.0\text{m}$).
* `base_link`: Main chassis box ($40 \times 28 \times 10\text{cm}$, mass $4.0\text{kg}$).
* 4 Wheels (`front_left`, `front_right`, `rear_left`, `rear_right`): Cylinders ($r = 0.05\text{m}$, width $0.04\text{m}$, continuous joints).
* Sensors: 2D LiDAR (`lidar_link`), Front Camera (`camera_link`), and 9-DOF IMU (`imu_link`).

```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/mobile_robot.urdf.xacro
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/mobile_robot.urdf.xacro
```
*Paste and save:*
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="four_wheel_robot">

  <!-- ==================================================================== -->
  <!-- 1. Robot Parameters and Dimensions                                   -->
  <!-- ==================================================================== -->
  <xacro:property name="chassis_length" value="0.40"/>
  <xacro:property name="chassis_width"  value="0.30"/>
  <xacro:property name="chassis_height" value="0.10"/>
  <xacro:property name="chassis_mass"   value="4.0"/>

  <xacro:property name="wheel_radius"   value="0.05"/>
  <xacro:property name="wheel_width"    value="0.04"/>
  <xacro:property name="wheel_mass"     value="0.4"/>

  <!-- Wheel Offsets -->
  <xacro:property name="wheel_x_offset" value="0.12"/>
  <xacro:property name="wheel_y_offset" value="0.17"/>
  <xacro:property name="wheel_z_offset" value="0.0"/>

  <!-- PI Constant -->
  <xacro:property name="PI" value="3.14159265359"/>

  <!-- ==================================================================== -->
  <!-- 2. Inertial Calculation Macros (For Gazebo Physics and Simulation)   -->
  <!-- ==================================================================== -->
  <xacro:macro name="box_inertia" params="m l w h">
    <inertial>
      <mass value="${m}"/>
      <inertia ixx="${(m/12.0) * (w*w + h*h)}" ixy="0.0" ixz="0.0"
               iyy="${(m/12.0) * (l*l + h*h)}" iyz="0.0"
               izz="${(m/12.0) * (l*l + w*w)}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertial>
      <mass value="${m}"/>
      <inertia ixx="${(m/12.0) * (3*r*r + h*h)}" ixy="0.0" ixz="0.0"
               iyy="${(m/12.0) * (3*r*r + h*h)}" iyz="0.0"
               izz="${(m/2.0) * (r*r)}"/>
    </inertial>
  </xacro:macro>

  <!-- ==================================================================== -->
  <!-- 3. Materials / Colors (High-Detail Photorealistic Aesthetics)        -->
  <!-- ==================================================================== -->
  <!-- Chassis Materials -->
  <material name="chassis_metal_blue">
    <color rgba="0.10 0.28 0.58 1.0"/> <!-- Metallic Sapphire Blue -->
  </material>

  <material name="carbon_top_deck">
    <color rgba="0.18 0.20 0.24 1.0"/> <!-- Carbon Fiber / Dark Titanium -->
  </material>

  <material name="bumper_matte_black">
    <color rgba="0.08 0.08 0.08 1.0"/> <!-- Tough Bumper Black -->
  </material>

  <!-- Wheel Materials -->
  <material name="tire_rubber_black">
    <color rgba="0.12 0.12 0.12 1.0"/> <!-- Matte Rubber Tire -->
  </material>

  <material name="wheel_alloy_silver">
    <color rgba="0.82 0.84 0.88 1.0"/> <!-- Brushed Aluminum Rim -->
  </material>

  <material name="wheel_hub_gold">
    <color rgba="0.85 0.65 0.15 1.0"/> <!-- Center Hub Nut Accent -->
  </material>

  <!-- LiDAR Sensor Materials -->
  <material name="lidar_color">
    <color rgba="0.85 0.15 0.15 1.0"/> <!-- Red -->
  </material>

  <material name="lidar_base_black">
    <color rgba="0.12 0.12 0.14 1.0"/> <!-- Anodized Black Base -->
  </material>

  <material name="lidar_optical_crimson">
    <color rgba="0.88 0.10 0.18 0.95"/> <!-- Optical Red Laser Turret -->
  </material>

  <material name="lidar_top_cap">
    <color rgba="0.05 0.05 0.05 1.0"/> <!-- Glossy Protective Cap -->
  </material>

  <!-- Camera Sensor Materials -->
  <material name="camera_casing_grey">
    <color rgba="0.16 0.18 0.22 1.0"/> <!-- Anodized Aluminum Casing -->
  </material>

  <material name="camera_lens_cyan">
    <color rgba="0.10 0.70 0.90 1.0"/> <!-- Optical Glass Lens -->
  </material>

  <material name="status_led_green">
    <color rgba="0.20 0.95 0.30 1.0"/> <!-- Power LED Indicator -->
  </material>

  <!-- IMU Sensor Materials -->
  <material name="imu_pcb_emerald">
    <color rgba="0.06 0.52 0.24 1.0"/> <!-- Classic Circuit Board Green -->
  </material>

  <material name="imu_chip_black">
    <color rgba="0.15 0.15 0.15 1.0"/> <!-- MEMS IC Microchip -->
  </material>

  <material name="imu_pin_gold">
    <color rgba="0.90 0.75 0.20 1.0"/> <!-- Gold-Plated Header Pins -->
  </material>

  <!-- ==================================================================== -->
  <!-- 4. Base Footprint (Ground Projection Link)                           -->
  <!-- ==================================================================== -->
  <link name="base_footprint"/>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- ==================================================================== -->
  <!-- 5. Base Link / Chassis (Multi-Deck Robot Architecture)               -->
  <!-- ==================================================================== -->
  <link name="base_link">
    <!-- 5A. Main Monocoque Chassis -->
    <visual>
      <origin xyz="0.0 0.0 0.025" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length} ${chassis_width} 0.09"/>
      </geometry>
      <material name="chassis_metal_blue"/>
    </visual>

    <!-- 5B. Top Equipment Deck Plate -->
    <visual>
      <origin xyz="0.0 0.0 0.075" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length - 0.02} ${chassis_width - 0.02} 0.01"/>
      </geometry>
      <material name="carbon_top_deck"/>
    </visual>

    <!-- 5C. Front Skid Bumper -->
    <visual>
      <origin xyz="${chassis_length/2 - 0.005} 0.0 0.02" rpy="0 0 0"/>
      <geometry>
        <box size="0.015 ${chassis_width} 0.05"/>
      </geometry>
      <material name="bumper_matte_black"/>
    </visual>

    <!-- Collision Geometry (Single Unified Bounding Box) -->
    <collision>
      <origin xyz="0.0 0.0 0.03" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
    </collision>
    <xacro:box_inertia m="${chassis_mass}" l="${chassis_length}" w="${chassis_width}" h="${chassis_height}"/>
  </link>

  <!-- ==================================================================== -->
  <!-- 6. Wheel Macro (Sports Rim Alloy + Rubber Tire)                      -->
  <!-- ==================================================================== -->
  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_wheel_link">
      <!-- 6A. Outer Rubber Tire -->
      <visual>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="tire_rubber_black"/>
      </visual>

      <!-- 6B. Brushed Aluminum Alloy Rim -->
      <visual>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius * 0.70}" length="${wheel_width + 0.002}"/>
        </geometry>
        <material name="wheel_alloy_silver"/>
      </visual>

      <!-- 6C. Center Hubcap Accent -->
      <visual>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius * 0.25}" length="${wheel_width + 0.004}"/>
        </geometry>
        <material name="wheel_hub_gold"/>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertia m="${wheel_mass}" r="${wheel_radius}" h="${wheel_width}"/>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${x_reflect * wheel_x_offset} ${y_reflect * wheel_y_offset} ${wheel_z_offset}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate the 4 Wheels -->
  <!-- Front Left -->
  <xacro:wheel prefix="front_left"  x_reflect="1"  y_reflect="1"/>
  <!-- Front Right -->
  <xacro:wheel prefix="front_right" x_reflect="1"  y_reflect="-1"/>
  <!-- Rear Left -->
  <xacro:wheel prefix="rear_left"   x_reflect="-1" y_reflect="1"/>
  <!-- Rear Right -->
  <xacro:wheel prefix="rear_right"  x_reflect="-1" y_reflect="-1"/>

  <!-- ==================================================================== -->
  <!-- 7. Sensors (Realistic RPLiDAR Puck, Vision Camera & IMU PCB)         -->
  <!-- ==================================================================== -->
  
  <!-- 7A. 2D LiDAR Sensor (Mounted flush on top surface of chassis - Z = 0.08 + 0.02 = 0.10m) -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.04"/>
      </geometry>
      <material name="lidar_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.04"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertia m="0.15" r="0.04" h="0.04"/>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.10 0.0 0.10" rpy="0 0 0"/>
  </joint>

  <!-- 7B. Front Camera (RealSense-Style Dual Optical Sensor) -->
  <link name="camera_link">
    <!-- Camera Body Casing -->
    <visual>
      <origin xyz="0.01 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.07 0.024"/>
      </geometry>
      <material name="camera_casing_grey"/>
    </visual>

    <!-- Optical Glass Lens (Center) -->
    <visual>
      <origin xyz="0.021 0.0 0" rpy="0 ${PI/2} 0"/>
      <geometry>
        <cylinder radius="0.007" length="0.003"/>
      </geometry>
      <material name="camera_lens_cyan"/>
    </visual>

    <!-- IR / Secondary Lens -->
    <visual>
      <origin xyz="0.021 0.022 0" rpy="0 ${PI/2} 0"/>
      <geometry>
        <cylinder radius="0.005" length="0.003"/>
      </geometry>
      <material name="lidar_optical_crimson"/>
    </visual>

    <!-- Power Indicator LED -->
    <visual>
      <origin xyz="0.021 -0.025 0.005" rpy="0 ${PI/2} 0"/>
      <geometry>
        <cylinder radius="0.002" length="0.002"/>
      </geometry>
      <material name="status_led_green"/>
    </visual>

    <collision>
      <origin xyz="0.01 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.07 0.024"/>
      </geometry>
    </collision>
    <xacro:box_inertia m="0.05" l="0.02" w="0.07" h="0.024"/>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.20 0.0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- 7C. IMU Sensor Link (MPU6050 / BNO055 Breakout Board) -->
  <link name="imu_link">
    <!-- PCB Substrate -->
    <visual>
      <origin xyz="0 0 0.001" rpy="0 0 0"/>
      <geometry>
        <box size="0.022 0.022 0.002"/>
      </geometry>
      <material name="imu_pcb_emerald"/>
    </visual>

    <!-- Microcontroller / MEMS IC Chip -->
    <visual>
      <origin xyz="0 0 0.003" rpy="0 0 0"/>
      <geometry>
        <box size="0.008 0.008 0.002"/>
      </geometry>
      <material name="imu_chip_black"/>
    </visual>

    <!-- Gold Pin Headers -->
    <visual>
      <origin xyz="-0.008 0 0.003" rpy="0 0 0"/>
      <geometry>
        <box size="0.003 0.018 0.002"/>
      </geometry>
      <material name="imu_pin_gold"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.001" rpy="0 0 0"/>
      <geometry>
        <box size="0.022 0.022 0.002"/>
      </geometry>
    </collision>
    <xacro:box_inertia m="0.01" l="0.022" w="0.022" h="0.002"/>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0.0 0.0 0.08" rpy="0 0 0"/>
  </joint>

  <!-- ==================================================================== -->
  <!-- 8. Gazebo Simulation Tags and Plugins                                -->
  <!-- ==================================================================== -->
  <xacro:include filename="mobile_robot_gazebo.xacro"/>

</robot>
```

### Step 1.4-A: Create Model Verification Launch File (`launch/display.launch.py`)
Provides intelligent dual modes in a single launch file:
* **Standalone URDF Inspection (`gui:=true`):** Automatically launches `robot_state_publisher`, `joint_state_publisher_gui` sliders, and `rviz2`.
* **Clean RViz Visualization (default `gui:=false`):** Opens RViz directly without popping up unwanted GUI joint sliders.
*(Note: Milestone 1 uses exclusively `display.launch.py`; no redundant launch files needed).*

```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_description/launch/display.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/launch/display.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Mobile Robot RViz Display Launch File (Intelligent Auto-Adaptive)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Intelligent Adaptive Execution:
  1. Standalone URDF Inspection Mode (Gazebo NOT running):
     ros2 launch mobile_robot_description display.launch.py
     -> Automatically launches robot_state_publisher, joint_state_publisher_gui
        sliders, and rviz2 for interactive mechanical model inspection.
  2. Simulation / Integrated Mode (Gazebo running in Terminal 1):
     ros2 launch mobile_robot_description display.launch.py
     -> Automatically detects active simulator (gzserver / ign / gz), synchronizes
        RViz to simulation clock (use_sim_time:=true), and avoids duplicate
        robot_state_publisher or conflicting GUI joint publisher nodes!
================================================================================
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def is_simulation_running():
    """Robustly detect if Gazebo Classic, Ignition, or Modern Gz is actively running."""
    my_pid = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == my_pid:
            continue
        try:
            with open(f'/proc/{pid}/comm', 'r') as f:
                comm = f.read().strip()
            if comm in ['bash', 'sh', 'python3', 'python']:
                continue
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().decode('utf-8', errors='ignore').replace('\x00', ' ')
            if any(token in cmd for token in ['gzserver', 'ign gazebo server', 'gz sim server', 'parameter_bridge']):
                return True
        except Exception:
            pass
    return False


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mobile_robot_description')
    
    gui_arg_val = LaunchConfiguration('gui').perform(context).lower()
    use_sim_time_arg_val = LaunchConfiguration('use_sim_time').perform(context).lower()
    run_rsp_arg_val = LaunchConfiguration('run_rsp').perform(context).lower()
    model_path = LaunchConfiguration('model').perform(context)
    sim_active = is_simulation_running()

    # 1. Determine use_sim_time
    if use_sim_time_arg_val in ['true', '1']:
        use_sim_time = True
    elif use_sim_time_arg_val in ['false', '0']:
        use_sim_time = False
    else:  # 'auto'
        use_sim_time = sim_active

    # 2. Determine gui (joint_state_publisher_gui)
    if gui_arg_val in ['true', '1']:
        enable_gui = True
    elif gui_arg_val in ['false', '0']:
        enable_gui = False
    else:  # 'auto'
        enable_gui = not sim_active

    # 3. Determine run_rsp (robot_state_publisher)
    if run_rsp_arg_val in ['true', '1']:
        enable_rsp = True
    elif run_rsp_arg_val in ['false', '0']:
        enable_rsp = False
    else:  # 'auto'
        # If Gazebo is running, gazebo.launch.py already provides robot_state_publisher
        enable_rsp = not sim_active

    # 4. Determine rviz config
    rviz_config_arg_val = LaunchConfiguration('rviz_config').perform(context)
    if rviz_config_arg_val.lower() in ['auto', 'default', '']:
        rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    elif rviz_config_arg_val.lower() in ['simulation', 'simulation.rviz', 'sim']:
        rviz_config_path = os.path.join(pkg_share, 'rviz', 'simulation.rviz')
    elif rviz_config_arg_val.lower() in ['display', 'display.rviz']:
        rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    else:
        rviz_config_path = rviz_config_arg_val

    # Process Xacro
    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str
    )

    nodes_to_launch = []

    # 1. Robot State Publisher (only if needed / not provided by gazebo)
    if enable_rsp:
        nodes_to_launch.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time
                }]
            )
        )

    # 2. Joint State Publisher GUI or fallback Joint State Publisher (only in standalone mode)
    if enable_gui:
        nodes_to_launch.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time
                }]
            )
        )
    elif not sim_active and enable_rsp:
        nodes_to_launch.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time
                }]
            )
        )

    # 3. RViz2 Node
    nodes_to_launch.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        )
    )

    return nodes_to_launch


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_description')
    
    default_model_path = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='auto',
        description='Enable joint_state_publisher_gui sliders: auto (true for standalone, false if Gazebo running), true, or false'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='auto',
        description='Use simulation (Gazebo) clock: auto (true if Gazebo running), true, or false'
    )
    
    run_rsp_arg = DeclareLaunchArgument(
        name='run_rsp',
        default_value='auto',
        description='Run robot_state_publisher: auto (false if Gazebo running), true, or false'
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf.xacro file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value='auto',
        description='Absolute path to rviz config file (auto-selects display.rviz or custom)'
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        run_rsp_arg,
        model_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])
```

### Step 1.4-B: Create Standalone & Simulation RViz Configurations

To ensure RViz always opens with the correct coordinate frames, Robot Model, TF, and sensor views pre-loaded without manual clicking:

> [!IMPORTANT]
> ### 🧭 Fixed Frame Rules Across Milestones
> * **Milestone 1 (URDF Preview & Standalone Gazebo):** Fixed Frame MUST be **`base_footprint`**.
>   * In Milestone 1, no motor controller is running yet, so there is NO `odom -> base_footprint` transform. If Fixed Frame is set to `odom` in Milestone 1, RViz will report: `Message Filter dropping message: frame 'lidar_link' ... queue is full` and the Robot Model cannot render.
>   * *Sensor Data Note:* In standalone URDF preview (`gui:=true`), the `Image` and `LaserScan` displays are idle because physical sensor nodes only publish when Gazebo or real camera drivers are running.
> * **Milestones 2–7 (Kinematics, Teleoperation & Navigation):** Fixed Frame MUST be **`odom`**.
>   * Starting in Milestone 2, the custom C++/Python kinematics node or `ros2_control diff_drive_controller` publishes the live `odom -> base_footprint` transform and `/odom` topic.

#### 1. Create Standalone RViz Configuration (`rviz/display.rviz` - Fixed Frame: `base_footprint`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_description/rviz/display.rviz
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/rviz/display.rviz
```
*Paste and save:*
```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /LaserScan1
        - /TF1
        - /Image1
      Splitter Ratio: 0.5
    Tree Height: 557
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        base_footprint:
          Value: false
        base_link:
          Value: false
        camera_link:
          Value: false
        front_left_wheel_link:
          Value: true
        front_right_wheel_link:
          Value: true
        imu_link:
          Value: false
        lidar_link:
          Value: false
        rear_left_wheel_link:
          Value: true
        rear_right_wheel_link:
          Value: true
      Marker Scale: 0.2
      Name: TF_Wheels
      Show Arrows: false
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        base_footprint:
          Value: false
        base_link:
          Value: false
        camera_link:
          Value: true
        front_left_wheel_link:
          Value: false
        front_right_wheel_link:
          Value: false
        imu_link:
          Value: true
        lidar_link:
          Value: true
        rear_left_wheel_link:
          Value: false
        rear_right_wheel_link:
          Value: false
      Marker Scale: 0.2
      Name: TF_Sensors
      Show Arrows: false
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: false
      Queue Size: 5
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /camera/image_raw
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_footprint
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 0; 200
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.8
      Focal Point:
        X: 0
        Y: 0
        Z: 0.1
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.45
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.78
```

#### 2. Create Simulation & Odometry RViz Configuration (`rviz/simulation.rviz` - Fixed Frame: `odom`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_description/rviz/simulation.rviz
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/rviz/simulation.rviz
```
*Paste and save:*
```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /LaserScan1
        - /TF1
        - /Odometry1
        - /Image1
        - /SafetyZoneMarkers1
      Splitter Ratio: 0.5
    Tree Height: 557
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        base_footprint:
          Value: false
        base_link:
          Value: false
        camera_link:
          Value: false
        front_left_wheel_link:
          Value: true
        front_right_wheel_link:
          Value: true
        imu_link:
          Value: false
        lidar_link:
          Value: false
        rear_left_wheel_link:
          Value: true
        rear_right_wheel_link:
          Value: true
      Marker Scale: 0.2
      Name: TF_Wheels
      Show Arrows: false
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        base_footprint:
          Value: false
        base_link:
          Value: false
        camera_link:
          Value: true
        front_left_wheel_link:
          Value: false
        front_right_wheel_link:
          Value: false
        imu_link:
          Value: true
        lidar_link:
          Value: true
        rear_left_wheel_link:
          Value: false
        rear_right_wheel_link:
          Value: false
      Marker Scale: 0.2
      Name: TF_Sensors
      Show Arrows: false
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/Odometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: false
        Position:
          Alpha: 0.3
          Color: 204; 51; 204
          Scale: 1
          Value: false
        Value: false
      Enabled: true
      Keep: 1
      Name: Odometry
      Position Tolerance: 0.1
      Queue Size: 10
      Shape:
        Alpha: 1
        Axes Length: 0.2
        Axes Radius: 0.02
        Color: 0; 255; 0
        Head Length: 0.1
        Head Radius: 0.05
        Shaft Length: 0.15
        Shaft Radius: 0.02
        Value: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /odom
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: false
      Queue Size: 5
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /camera/image_raw
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: SafetyZoneMarkers
      Namespaces:
        {}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /safety_zone_markers
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 0; 200
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.5
      Focal Point:
        X: 0
        Y: 0
        Z: 0.1
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: <Fixed Frame>
```

### Step 1.5: Adding Simulation Physics & Sensors (`urdf/mobile_robot_gazebo.xacro`)
> [!NOTE]
> **Why do we need a separate `mobile_robot_gazebo.xacro`?**  
> Pure URDF only defines geometry and kinematics for RViz. Gazebo requires physical simulation properties:
> 1. **Surface Contact Friction:** Tuned for 4-wheel differential drive (`mu1: 0.8` rolling traction, `mu2: 0.1` lateral slip, `<fdir1>1 0 0</fdir1>`, `<maxVel>0.5</maxVel>`, `<kp>100000.0</kp>`, `<kd>1.0</kd>`) to eliminate wheel bounce and friction chatter.
> 2. **Sensor Simulation Plugins:** 2D LiDAR (`/scan`), Front Camera (`/camera/image_raw`), and IMU (`/imu/data`) for both Gazebo Classic and Ignition / Modern Gz.  
> 
> *(Notice: Milestone 1 contains **strictly surface physics and physical sensors**. No motor controllers, diff_drive plugins, or joint publishers are added here — those are introduced in Milestone 2).*

```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/mobile_robot_gazebo.xacro
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/mobile_robot_gazebo.xacro
```
*Paste and save:*
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ==================================================================== -->
  <!-- 1. Gazebo Colors and Surface Friction Properties                     -->
  <!-- ==================================================================== -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="lidar_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="camera_link">
    <material>Gazebo/Yellow</material>
  </gazebo>

  <!-- Wheel Friction Properties -->
  <xacro:macro name="wheel_gazebo" params="prefix">
    <gazebo reference="${prefix}_wheel_link">
      <material>Gazebo/DarkGrey</material>
      <mu1>0.8</mu1>
      <mu2>0.1</mu2>
      <fdir1>1 0 0</fdir1>
      <kp>100000.0</kp>
      <kd>1.0</kd>
      <minDepth>0.001</minDepth>
      <maxVel>0.5</maxVel>
    </gazebo>
  </xacro:macro>

  <xacro:wheel_gazebo prefix="front_left"/>
  <xacro:wheel_gazebo prefix="front_right"/>
  <xacro:wheel_gazebo prefix="rear_left"/>
  <xacro:wheel_gazebo prefix="rear_right"/>

  <!-- ==================================================================== -->
  <!-- 2. Simulation Plugins (Gazebo Classic vs Ignition / Modern Gazebo)   -->
  <!-- ==================================================================== -->
  <xacro:arg name="is_ignition" default="false"/>
  <xacro:arg name="use_ros2_control" default="false"/>

  <!-- ==================================================================== -->
  <!-- 2A. GAZEBO CLASSIC (Gazebo 11) PLUGINS (Only when NOT Ignition)      -->
  <!-- ==================================================================== -->
  <xacro:unless value="$(arg is_ignition)">

    <!-- Gazebo Classic Joint State Publisher (Publishes all 4 wheel joint transforms) -->
    <gazebo>
      <plugin name="gazebo_joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
        <ros>
          <remapping>~/out:=/joint_states</remapping>
        </ros>
        <update_rate>30</update_rate>
        <joint_name>front_left_wheel_joint</joint_name>
        <joint_name>front_right_wheel_joint</joint_name>
        <joint_name>rear_left_wheel_joint</joint_name>
        <joint_name>rear_right_wheel_joint</joint_name>
      </plugin>
    </gazebo>

    <!-- Gazebo Classic 2D LiDAR Ray Sensor Plugin -->
    <gazebo reference="lidar_link">
      <sensor name="lidar_sensor" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>10.0</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.12</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <remapping>~/out:=/scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>lidar_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>

    <!-- Gazebo Classic Front Camera Sensor Plugin -->
    <gazebo reference="camera_link">
      <sensor name="front_camera" type="camera">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>30.0</update_rate>
        <camera>
          <horizontal_fov>1.089</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>8.0</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <remapping>~/image_raw:=/camera/image_raw</remapping>
            <remapping>~/camera_info:=/camera/camera_info</remapping>
          </ros>
          <camera_name>camera</camera_name>
          <frame_name>camera_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>

    <!-- Gazebo Classic IMU Sensor Plugin -->
    <gazebo reference="imu_link">
      <material>Gazebo/Green</material>
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>50.0</update_rate>
        <visualize>false</visualize>
        <topic>/imu/data</topic>
        <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
          <ros>
            <namespace>/</namespace>
            <remapping>~/out:=/imu/data</remapping>
          </ros>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
          <frame_name>imu_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>

  </xacro:unless>

  <!-- ==================================================================== -->
  <!-- 2B. IGNITION GAZEBO (ign) & MODERN GAZEBO (gz) SYSTEMS & PLUGINS     -->
  <!-- ==================================================================== -->
  <xacro:if value="$(arg is_ignition)">
    <gazebo>
      <!-- Joint State Publisher for Ignition / Modern Gazebo -->
      <plugin filename="libignition-gazebo-joint-state-publisher-system.so" name="ignition::gazebo::systems::JointStatePublisher">
        <topic>/joint_states</topic>
      </plugin>

      <!-- Sensors System for Ignition / Modern Gazebo -->
      <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
        <render_engine>ogre2</render_engine>
      </plugin>

      <!-- IMU System for Ignition / Modern Gazebo -->
      <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu"/>
    </gazebo>

    <!-- Ignition / Modern Gz Sensors -->
    <gazebo reference="lidar_link">
      <sensor name="lidar_sensor_ign" type="gpu_lidar">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>10.0</update_rate>
        <topic>/scan</topic>
        <gz_frame_id>lidar_link</gz_frame_id>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.12</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
      </sensor>
    </gazebo>

    <gazebo reference="camera_link">
      <sensor name="front_camera_ign" type="camera">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>30.0</update_rate>
        <topic>/camera/image_raw</topic>
        <gz_frame_id>camera_link</gz_frame_id>
        <camera>
          <horizontal_fov>1.089</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>8.0</far>
          </clip>
        </camera>
      </sensor>
    </gazebo>
  </xacro:if>

</robot>
```

### Step 1.6: Create Gazebo Physics Launch Files
Create the Gazebo Classic physics launcher (`launch/gazebo.launch.py`):
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_description/launch/gazebo.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/launch/gazebo.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_desc = get_package_share_directory('mobile_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    default_model_path = os.path.join(pkg_desc, 'urdf', 'mobile_robot.urdf.xacro')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf.xacro file'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='true',
        description='Enable ros2_control plugin (true) or classic diff_drive plugin (false)'
    )

    # Robot Description
    robot_description = ParameterValue(
        Command([
            'xacro ', LaunchConfiguration('model'),
            ' is_ignition:=false',
            ' use_ros2_control:=', LaunchConfiguration('use_ros2_control')
        ]),
        value_type=str
    )

    # 1. Robot State Publisher (with use_sim_time=True for Gazebo)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo Server & Client (empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    # 3. Spawn Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'four_wheel_robot',
            '-z', '0.05'
        ],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        use_ros2_control_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity
    ])
```

Create the Ignition / Modern Gz launch file (`launch/ign_gazebo.launch.py`):
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_description/launch/ign_gazebo.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/launch/ign_gazebo.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Ignition Gazebo (ign) & Modern Gazebo (gz) Simulation Launch File
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Launches Ignition Gazebo / Modern Gz Sim with:
 1. Robot State Publisher (URDF/Xacro)
 2. Gazebo Sim Server & GUI (via ros_gz_sim)
 3. Entity Spawner (via ros_gz_sim create)
 4. ROS-GZ Bridge (bridges cmd_vel, odom, tf, scan, camera, imu, clock)
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('mobile_robot_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'mobile_robot.urdf.xacro')

    gz_args_arg = DeclareLaunchArgument(
        'gz_args',
        default_value='-r empty.sdf',
        description='Arguments to be passed to Gazebo Sim / Ignition'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Enable ros2_control plugin (true) or classic diff_drive plugin (false)'
    )

    # 1. Robot Description from Xacro (with is_ignition:=true)
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' is_ignition:=true',
            ' use_ros2_control:=', LaunchConfiguration('use_ros2_control')
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo Sim / Ignition launcher
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    # 3. Entity Spawner
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'four_wheel_robot',
            '-z', '0.05'
        ]
    )

    # 4. ROS-GZ Parameter Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ]
    )

    return LaunchDescription([
        gz_args_arg,
        use_ros2_control_arg,
        robot_state_publisher_node,
        gz_sim_launch,
        spawn_node,
        bridge_node,
    ])
```

### Step 1.7: Edit `CMakeLists.txt` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/CMakeLists.txt
```
*Paste and save:*
```cmake
cmake_minimum_required(VERSION 3.8)
project(mobile_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### Step 1.8: Edit `package.xml` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/package.xml
```
*Paste and save:*
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mobile_robot_description</name>
  <version>1.0.0</version>
  <description>Robot description package (URDF/Xacro, Gazebo, RViz, Camera, LiDAR) for 4-wheel robot.</description>
  <maintainer email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</maintainer>
  <author email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</author>
  <license>Apache-2.0</license>

  <url type="website">https://rahul-r-joshua.github.io/portfolio/</url>
  <url type="repository">https://github.com/rahul-r-joshua</url>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 1.9: Build `mobile_robot_description`
```bash
cd ~/ros2_mobile_robot_ws
colcon build --packages-select mobile_robot_description --symlink-install
source install/setup.bash
```

---

### ⚡ MILESTONE 1 VERIFICATION

> [!IMPORTANT]
> **⚠️ Beginner Rule: In EVERY new terminal tab you open, always run:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_mobile_robot_ws/install/setup.bash
> ```

#### Part 0: URDF XML & Kinematic Tree Validation (`check_urdf`)
Always validate the generated URDF XML syntax and link/joint tree before launching visualizers or physics engines:
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
xacro src/mobile_robot_description/urdf/mobile_robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```
*Expected output:* `Successfully Parsed XML` with `root Link: base_footprint` and child links for chassis, 4 wheels, LiDAR, camera, and IMU.

#### Part 1: Visual Verification in Standalone RViz (URDF Mechanical & Kinematics Preview)
Open RViz to inspect the 3D mechanical chassis, 4 wheels, and sensor mounting positions:

```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_description display.launch.py
```
*(The interactive `joint_state_publisher_gui` sliders window opens automatically. Drag the 4 wheel sliders to rotate the wheels and verify the kinematic tree interactively in RViz!)*

> [!TIP]
> ### 🎛️ Manual RViz Display Configuration (If starting from an empty RViz window):
> If RViz opens without the robot model or displays pre-loaded, configure the displays manually in 4 simple steps:
> 1. **Set Fixed Frame:** In the left **Displays** panel under **Global Options**, change **Fixed Frame** to `base_footprint`.
> 2. **Add Robot Model:** Click **Add** (bottom left) ➔ Select **By display type** tab ➔ Choose **RobotModel** ➔ Click OK. Under RobotModel properties, set **Description Topic** to `/robot_description`.
> 3. **Add Coordinate Axes (TF):** Click **Add** ➔ Choose **TF** ➔ Click OK. Check **Show Axes** and **Show Names** to see the coordinate frames of all 4 wheels, LiDAR, camera, and IMU.
> 4. **Add Sensor Displays:** Click **Add** ➔ Choose **LaserScan** (Topic: `/scan`, Size: `0.05`) and **Image** (Topic: `/camera/image_raw`).
>
> ❓ **Why are sensors (LiDAR & Camera) idle in Milestone 1 Standalone RViz?**  
> In Milestone 1 Part 1, you are running RViz in **standalone URDF inspection mode** without the physics simulator. Gazebo is not running yet, so no simulated laser rays or camera frames exist. Once you launch Gazebo in Part 2, the simulated sensors will actively publish data to `/scan` and `/camera/image_raw`, and the red laser points and camera stream will instantly appear in RViz!

#### Part 2: Physics Simulation Spawn & Live Sensor Verification
Verify that the robot model with physics parameters, surface friction, and sensor plugins spawns cleanly into your simulation engine and streams sensor data to RViz:

##### Terminal 1: Launch Physics Simulator (Choose One Backend)

* **Option A: Gazebo Classic (Gazebo 11) Physics World:**
```bash
cd ~/ros2_mobile_robot_ws
source /usr/share/gazebo/setup.bash
source install/setup.bash
ros2 launch mobile_robot_description gazebo.launch.py
```

* **Option B: Ignition Gazebo (Fortress) Physics World:**
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_description ign_gazebo.launch.py
```

* **Option C: Modern Gazebo (Gz Sim / Harmonic / Garden) Physics World:**
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_description ign_gazebo.launch.py backend:=gz
```

> [!TIP]
> **Gazebo Physics Checklist:**  
> 1. Gazebo opens with an empty world ground plane.  
> 2. The entity `four_wheel_robot` is spawned at $Z = 0.05\text{m}$ and drops cleanly onto the ground plane without vibrating, clipping through the floor, or toppling over.  
> 3. Check the terminal log to confirm `Spawn status: Successfully spawned entity [four_wheel_robot]`.

##### Terminal 2: Launch RViz Visualizer (Display Robot Model & Live Sensors)
In a separate terminal, open RViz to display the robot model, live 2D LiDAR pointcloud, and front camera feed synchronized with the simulation clock:

```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_description display.launch.py
```

> [!NOTE]
> 🧠 **Why are GUI joint sliders disabled during Gazebo simulation?**  
> In physics simulation (Part 2), wheel positions and velocities are governed dynamically by the physics engine, contact friction, and motor controllers (`/cmd_vel` in Milestone 2). Manual kinematic slider overrides are automatically disabled to prevent conflict with Gazebo's live physics joint state stream.

> [!TIP]
> 🔴 **Seeing LaserScan Points in Gazebo & RViz:**  
> In an empty Gazebo world, LiDAR rays shoot out into empty space, so no obstacle returns are measured (ranges return `inf`).  
> * To visualize active laser rays and points:  
>   1. In the **Gazebo top toolbar**, click the **Cube (Box)** or **Cylinder** icon.  
>   2. Click on the ground plane in front of the robot ($0.5\text{m}$ to $2.0\text{m}$ away) to drop the obstacle.  
>   3. In **RViz**, bright red point reflections will immediately appear on the surface of the object from `/scan`!  
>   4. You can adjust the **Size (m)** property under LaserScan in RViz (e.g. `0.05`) for clear visibility.

*Check active topics in another terminal:*
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```
*Expected output includes:*
* `/clock` *(Simulation clock)*
* `/robot_description` *(URDF model stream)*
* `/tf_static` *(Static link transforms)*
* `/scan` *(2D LiDAR LaserScan output)*
* `/camera/image_raw` *(RGB Camera stream)*
* `/imu/data` *(9-axis IMU sensor output)*

#### Part 3: Transform Verification (`tf2_echo`) & Lookup Hardening
Verify live transformations between links in the kinematic chain (ensure `display.launch.py` or `gazebo.launch.py` is running in another terminal):
```bash
# Verify transform with a 5-second non-blocking timeout:
timeout 5s ros2 run tf2_ros tf2_echo base_footprint lidar_link || true
```
*(Note: Running `ros2 run tf2_ros tf2_echo` without `timeout 5s` streams continuously. Press `Ctrl+C` to return to your command prompt).*


> [!TIP]
> ### 🛡️ Transform Lookup Hardening (Preventing Startup Race Conditions)
> In ROS 2, when nodes query transforms via `tf2_ros::Buffer`, lookups can fail during the first 1–2 seconds if `lookup_transform` is called without a timeout or without exception handling.
> 
> **C++ (`rclcpp`):** Always specify a timeout (e.g. `50ms`–`100ms`) and catch `tf2::TransformException`:
> ```cpp
> try {
>   auto tf = tf_buffer_->lookup_transform("odom", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.1));
> } catch (const tf2::TransformException & ex) {
>   RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF lookup waiting: %s", ex.what());
> }
> ```
> 
> **Python (`rclpy`):**
> ```python
> from tf2_ros import TransformException
> from rclpy.duration import Duration
> 
> try:
>     tf = self.tf_buffer.lookup_transform("odom", "base_footprint", Time(), timeout=Duration(seconds=0.1))
> except TransformException as ex:
>     self.get_logger().warn(f"TF lookup waiting: {ex}", throttle_duration_sec=1.0)
> ```

---

<a id="milestone-2" name="milestone-2"></a>
## Milestone 2
### Differential Drive Kinematics & Odometry (`mobile_robot_controller`)

In Milestone 2, you transition from static 3D robot modeling to dynamic motion control and odometry estimation.

> [!NOTE]
> **Why is Control Separated from Gazebo?**  
> Gazebo simulates the physical environment (gravity, wheel friction, sensor beams). The control stack runs as separate ROS 2 nodes that calculate kinematics and command wheel velocities. Separating simulation from control ensures the exact same controller code works in Gazebo simulation AND on physical robot hardware!

### 🌉 Prerequisite Step 2.0: Bridge Description to Actuation (Updating `mobile_robot_description`)

Before implementing controllers, we must equip our robot description with hardware control interfaces (`ros2_control.xacro` and `transmission.xacro`), and configure the Gazebo controller plugin.

#### Step 2.0.1: Create Hardware Interface System (`urdf/ros2_control.xacro`)
Defines the `ros2_control` hardware system interface for commanding wheel velocities and reading positions:
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/ros2_control.xacro
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/ros2_control.xacro
```
*Paste and save:*
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ==================================================================== -->
  <!-- ros2_control System Interface for 4-Wheel Differential Drive Robot    -->
  <!-- ==================================================================== -->
  <xacro:arg name="is_ignition" default="false"/>

  <ros2_control name="MobileRobotHardwareSystem" type="system">
    <hardware>
      <xacro:if value="$(arg is_ignition)">
        <plugin>ign_ros2_control/IgnitionSystem</plugin>
      </xacro:if>
      <xacro:unless value="$(arg is_ignition)">
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
      </xacro:unless>
    </hardware>

    <!-- Front Left Wheel Joint -->
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10.0</param>
        <param name="max">10.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <!-- Front Right Wheel Joint -->
    <joint name="front_right_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10.0</param>
        <param name="max">10.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <!-- Rear Left Wheel Joint -->
    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10.0</param>
        <param name="max">10.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <!-- Rear Right Wheel Joint -->
    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10.0</param>
        <param name="max">10.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

</robot>
```

#### Step 2.0.2: Create Transmissions (`urdf/transmission.xacro`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/transmission.xacro
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/transmission.xacro
```
*Paste and save:*
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ==================================================================== -->
  <!-- Transmissions & Actuators for 4-Wheel Differential Drive Robot       -->
  <!-- ==================================================================== -->
  <xacro:macro name="wheel_transmission" params="prefix">
    <transmission name="${prefix}_wheel_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${prefix}_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      </joint>
      <actuator name="${prefix}_wheel_motor">
        <mechanicalReduction>1.0</mechanicalReduction>
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      </actuator>
    </transmission>
  </xacro:macro>

  <xacro:wheel_transmission prefix="front_left"/>
  <xacro:wheel_transmission prefix="front_right"/>
  <xacro:wheel_transmission prefix="rear_left"/>
  <xacro:wheel_transmission prefix="rear_right"/>

</robot>
```

#### Step 2.0.3: Update Master URDF (`urdf/mobile_robot.urdf.xacro`)
Open `urdf/mobile_robot.urdf.xacro` with `gedit`:
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/mobile_robot.urdf.xacro
```
Scroll to the bottom of the file, and right before the closing `</robot>` tag, add Section 9:
```xml
  <!-- ==================================================================== -->
  <!-- 9. ros2_control and Transmissions                                    -->
  <!-- ==================================================================== -->
  <xacro:include filename="$(find mobile_robot_description)/urdf/ros2_control.xacro"/>
  <xacro:include filename="$(find mobile_robot_description)/urdf/transmission.xacro"/>
```

*If you prefer to replace the entire file at once, you can use this complete updated URDF:*
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="four_wheel_robot">

  <!-- ==================================================================== -->
  <!-- 1. Robot Parameters and Dimensions                                   -->
  <!-- ==================================================================== -->
  <xacro:property name="chassis_length" value="0.40"/>
  <xacro:property name="chassis_width"  value="0.30"/>
  <xacro:property name="chassis_height" value="0.10"/>
  <xacro:property name="chassis_mass"   value="4.0"/>

  <xacro:property name="wheel_radius"   value="0.05"/>
  <xacro:property name="wheel_width"    value="0.04"/>
  <xacro:property name="wheel_mass"     value="0.4"/>

  <!-- Wheel Offsets -->
  <xacro:property name="wheel_x_offset" value="0.12"/>
  <xacro:property name="wheel_y_offset" value="0.17"/>
  <xacro:property name="wheel_z_offset" value="0.0"/>

  <!-- PI Constant -->
  <xacro:property name="PI" value="3.14159265359"/>

  <!-- ==================================================================== -->
  <!-- 2. Inertial Calculation Macros (For Gazebo Physics and Simulation)   -->
  <!-- ==================================================================== -->
  <xacro:macro name="box_inertia" params="m l w h">
    <inertial>
      <mass value="${m}"/>
      <inertia ixx="${(m/12.0) * (w*w + h*h)}" ixy="0.0" ixz="0.0"
               iyy="${(m/12.0) * (l*l + h*h)}" iyz="0.0"
               izz="${(m/12.0) * (l*l + w*w)}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertial>
      <mass value="${m}"/>
      <inertia ixx="${(m/12.0) * (3*r*r + h*h)}" ixy="0.0" ixz="0.0"
               iyy="${(m/12.0) * (3*r*r + h*h)}" iyz="0.0"
               izz="${(m/2.0) * (r*r)}"/>
    </inertial>
  </xacro:macro>

  <!-- ==================================================================== -->
  <!-- 3. Materials / Colors (High-Detail Photorealistic Aesthetics)        -->
  <!-- ==================================================================== -->
  <!-- Chassis Materials -->
  <material name="chassis_metal_blue">
    <color rgba="0.10 0.28 0.58 1.0"/> <!-- Metallic Sapphire Blue -->
  </material>

  <material name="carbon_top_deck">
    <color rgba="0.18 0.20 0.24 1.0"/> <!-- Carbon Fiber / Dark Titanium -->
  </material>

  <material name="bumper_matte_black">
    <color rgba="0.08 0.08 0.08 1.0"/> <!-- Tough Bumper Black -->
  </material>

  <!-- Wheel Materials -->
  <material name="tire_rubber_black">
    <color rgba="0.12 0.12 0.12 1.0"/> <!-- Matte Rubber Tire -->
  </material>

  <material name="wheel_alloy_silver">
    <color rgba="0.82 0.84 0.88 1.0"/> <!-- Brushed Aluminum Rim -->
  </material>

  <material name="wheel_hub_gold">
    <color rgba="0.85 0.65 0.15 1.0"/> <!-- Center Hub Nut Accent -->
  </material>

  <!-- LiDAR Sensor Materials -->
  <material name="lidar_color">
    <color rgba="0.85 0.15 0.15 1.0"/> <!-- Red -->
  </material>

  <material name="lidar_base_black">
    <color rgba="0.12 0.12 0.14 1.0"/> <!-- Anodized Black Base -->
  </material>

  <material name="lidar_optical_crimson">
    <color rgba="0.88 0.10 0.18 0.95"/> <!-- Optical Red Laser Turret -->
  </material>

  <material name="lidar_top_cap">
    <color rgba="0.05 0.05 0.05 1.0"/> <!-- Glossy Protective Cap -->
  </material>

  <!-- Camera Sensor Materials -->
  <material name="camera_casing_grey">
    <color rgba="0.16 0.18 0.22 1.0"/> <!-- Anodized Aluminum Casing -->
  </material>

  <material name="camera_lens_cyan">
    <color rgba="0.10 0.70 0.90 1.0"/> <!-- Optical Glass Lens -->
  </material>

  <material name="status_led_green">
    <color rgba="0.20 0.95 0.30 1.0"/> <!-- Power LED Indicator -->
  </material>

  <!-- IMU Sensor Materials -->
  <material name="imu_pcb_emerald">
    <color rgba="0.06 0.52 0.24 1.0"/> <!-- Classic Circuit Board Green -->
  </material>

  <material name="imu_chip_black">
    <color rgba="0.15 0.15 0.15 1.0"/> <!-- MEMS IC Microchip -->
  </material>

  <material name="imu_pin_gold">
    <color rgba="0.90 0.75 0.20 1.0"/> <!-- Gold-Plated Header Pins -->
  </material>

  <!-- ==================================================================== -->
  <!-- 4. Base Footprint (Ground Projection Link)                           -->
  <!-- ==================================================================== -->
  <link name="base_footprint"/>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- ==================================================================== -->
  <!-- 5. Base Link / Chassis (Multi-Deck Robot Architecture)               -->
  <!-- ==================================================================== -->
  <link name="base_link">
    <!-- 5A. Main Monocoque Chassis -->
    <visual>
      <origin xyz="0.0 0.0 0.025" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length} ${chassis_width} 0.09"/>
      </geometry>
      <material name="chassis_metal_blue"/>
    </visual>

    <!-- 5B. Top Equipment Deck Plate -->
    <visual>
      <origin xyz="0.0 0.0 0.075" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length - 0.02} ${chassis_width - 0.02} 0.01"/>
      </geometry>
      <material name="carbon_top_deck"/>
    </visual>

    <!-- 5C. Front Skid Bumper -->
    <visual>
      <origin xyz="${chassis_length/2 - 0.005} 0.0 0.02" rpy="0 0 0"/>
      <geometry>
        <box size="0.015 ${chassis_width} 0.05"/>
      </geometry>
      <material name="bumper_matte_black"/>
    </visual>

    <!-- Collision Geometry (Single Unified Bounding Box) -->
    <collision>
      <origin xyz="0.0 0.0 0.03" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
    </collision>
    <xacro:box_inertia m="${chassis_mass}" l="${chassis_length}" w="${chassis_width}" h="${chassis_height}"/>
  </link>

  <!-- ==================================================================== -->
  <!-- 6. Wheel Macro (Sports Rim Alloy + Rubber Tire)                      -->
  <!-- ==================================================================== -->
  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_wheel_link">
      <!-- 6A. Outer Rubber Tire -->
      <visual>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="tire_rubber_black"/>
      </visual>

      <!-- 6B. Brushed Aluminum Alloy Rim -->
      <visual>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius * 0.70}" length="${wheel_width + 0.002}"/>
        </geometry>
        <material name="wheel_alloy_silver"/>
      </visual>

      <!-- 6C. Center Hubcap Accent -->
      <visual>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius * 0.25}" length="${wheel_width + 0.004}"/>
        </geometry>
        <material name="wheel_hub_gold"/>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertia m="${wheel_mass}" r="${wheel_radius}" h="${wheel_width}"/>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${x_reflect * wheel_x_offset} ${y_reflect * wheel_y_offset} ${wheel_z_offset}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate the 4 Wheels -->
  <!-- Front Left -->
  <xacro:wheel prefix="front_left"  x_reflect="1"  y_reflect="1"/>
  <!-- Front Right -->
  <xacro:wheel prefix="front_right" x_reflect="1"  y_reflect="-1"/>
  <!-- Rear Left -->
  <xacro:wheel prefix="rear_left"   x_reflect="-1" y_reflect="1"/>
  <!-- Rear Right -->
  <xacro:wheel prefix="rear_right"  x_reflect="-1" y_reflect="-1"/>

  <!-- ==================================================================== -->
  <!-- 7. Sensors (Realistic RPLiDAR Puck, Vision Camera & IMU PCB)         -->
  <!-- ==================================================================== -->
  
  <!-- 7A. 2D LiDAR Sensor (Mounted flush on top surface of chassis - Z = 0.08 + 0.02 = 0.10m) -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.04"/>
      </geometry>
      <material name="lidar_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.04"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertia m="0.15" r="0.04" h="0.04"/>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.10 0.0 0.10" rpy="0 0 0"/>
  </joint>

  <!-- 7B. Front Camera (RealSense-Style Dual Optical Sensor) -->
  <link name="camera_link">
    <!-- Camera Body Casing -->
    <visual>
      <origin xyz="0.01 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.07 0.024"/>
      </geometry>
      <material name="camera_casing_grey"/>
    </visual>

    <!-- Optical Glass Lens (Center) -->
    <visual>
      <origin xyz="0.021 0.0 0" rpy="0 ${PI/2} 0"/>
      <geometry>
        <cylinder radius="0.007" length="0.003"/>
      </geometry>
      <material name="camera_lens_cyan"/>
    </visual>

    <!-- IR / Secondary Lens -->
    <visual>
      <origin xyz="0.021 0.022 0" rpy="0 ${PI/2} 0"/>
      <geometry>
        <cylinder radius="0.005" length="0.003"/>
      </geometry>
      <material name="lidar_optical_crimson"/>
    </visual>

    <!-- Power Indicator LED -->
    <visual>
      <origin xyz="0.021 -0.025 0.005" rpy="0 ${PI/2} 0"/>
      <geometry>
        <cylinder radius="0.002" length="0.002"/>
      </geometry>
      <material name="status_led_green"/>
    </visual>

    <collision>
      <origin xyz="0.01 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.07 0.024"/>
      </geometry>
    </collision>
    <xacro:box_inertia m="0.05" l="0.02" w="0.07" h="0.024"/>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.20 0.0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- 7C. IMU Sensor Link (MPU6050 / BNO055 Breakout Board) -->
  <link name="imu_link">
    <!-- PCB Substrate -->
    <visual>
      <origin xyz="0 0 0.001" rpy="0 0 0"/>
      <geometry>
        <box size="0.022 0.022 0.002"/>
      </geometry>
      <material name="imu_pcb_emerald"/>
    </visual>

    <!-- Microcontroller / MEMS IC Chip -->
    <visual>
      <origin xyz="0 0 0.003" rpy="0 0 0"/>
      <geometry>
        <box size="0.008 0.008 0.002"/>
      </geometry>
      <material name="imu_chip_black"/>
    </visual>

    <!-- Gold Pin Headers -->
    <visual>
      <origin xyz="-0.008 0 0.003" rpy="0 0 0"/>
      <geometry>
        <box size="0.003 0.018 0.002"/>
      </geometry>
      <material name="imu_pin_gold"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.001" rpy="0 0 0"/>
      <geometry>
        <box size="0.022 0.022 0.002"/>
      </geometry>
    </collision>
    <xacro:box_inertia m="0.01" l="0.022" w="0.022" h="0.002"/>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0.0 0.0 0.08" rpy="0 0 0"/>
  </joint>

  <!-- ==================================================================== -->
  <!-- 8. Gazebo Simulation Tags and Plugins                                -->
  <!-- ==================================================================== -->
  <xacro:include filename="mobile_robot_gazebo.xacro"/>

  <!-- ==================================================================== -->
  <!-- 9. ros2_control and Transmissions                                    -->
  <!-- ==================================================================== -->
  <xacro:include filename="ros2_control.xacro"/>
  <xacro:include filename="transmission.xacro"/>

</robot>
```

#### Step 2.0.4: Update Simulation Plugins (`urdf/mobile_robot_gazebo.xacro`)
Open `urdf/mobile_robot_gazebo.xacro` with `gedit`:
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/mobile_robot_gazebo.xacro
```

Now that you are building the controller stack in Milestone 2, add the controller plugins for your simulation backends:

##### 1. For Gazebo Classic (`gazebo_ros2_control`):
Add the `gazebo_ros2_control` plugin under `<xacro:unless value="$(arg is_ignition)">`, right above the sensor plugins:
```xml
  <!-- ==================================================================== -->
  <!-- 2A. ros2_control Gazebo System Plugin (Gazebo Classic)               -->
  <!-- ==================================================================== -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find mobile_robot_controller)/config/ros2_robot_controller_params.yaml</parameters>
      <ros>
        <remapping>/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel</remapping>
        <remapping>/diff_drive_controller/odom:=/odom</remapping>
      </ros>
    </plugin>
  </gazebo>
```

##### 2. For Ignition Gazebo / Modern Gz (`DiffDrive` & `JointStatePublisher`):
Add the `DiffDrive` and `JointStatePublisher` system plugins under `<xacro:if value="$(arg is_ignition)">`:
```xml
  <!-- ==================================================================== -->
  <!-- 2B. Diff Drive & Joint State Systems (Ignition / Modern Gz)          -->
  <!-- ==================================================================== -->
  <gazebo>
    <!-- Diff Drive System for Ignition / Modern Gazebo -->
    <plugin filename="libignition-gazebo-diff-drive-system.so" name="ignition::gazebo::systems::DiffDrive">
      <left_joint>front_left_wheel_joint</left_joint>
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <wheel_separation>0.34</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <odom_publish_frequency>50</odom_publish_frequency>
      <topic>/cmd_vel</topic>
      <odom_topic>/odom</odom_topic>
      <tf_topic>/tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
    </plugin>

    <!-- Joint State Publisher for Ignition / Modern Gazebo -->
    <plugin filename="libignition-gazebo-joint-state-publisher-system.so" name="ignition::gazebo::systems::JointStatePublisher">
      <topic>/joint_states</topic>
    </plugin>
  </gazebo>
```

*If you prefer to replace the entire file at once, you can use this complete updated `mobile_robot_gazebo.xacro` containing all simulation systems:*
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ==================================================================== -->
  <!-- 1. Gazebo Colors and Surface Friction Properties                     -->
  <!-- ==================================================================== -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="lidar_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="camera_link">
    <material>Gazebo/Yellow</material>
  </gazebo>

  <!-- Wheel Friction Properties -->
  <xacro:macro name="wheel_gazebo" params="prefix">
    <gazebo reference="${prefix}_wheel_link">
      <material>Gazebo/DarkGrey</material>
      <mu1>0.8</mu1>
      <mu2>0.1</mu2>
      <fdir1>1 0 0</fdir1>
      <kp>100000.0</kp>
      <kd>1.0</kd>
      <minDepth>0.001</minDepth>
      <maxVel>0.5</maxVel>
    </gazebo>
  </xacro:macro>

  <xacro:wheel_gazebo prefix="front_left"/>
  <xacro:wheel_gazebo prefix="front_right"/>
  <xacro:wheel_gazebo prefix="rear_left"/>
  <xacro:wheel_gazebo prefix="rear_right"/>

  <!-- ==================================================================== -->
  <!-- 2. Simulation Plugins (Gazebo Classic vs Ignition / Modern Gazebo)   -->
  <!-- ==================================================================== -->
  <xacro:arg name="is_ignition" default="false"/>
  <xacro:arg name="use_ros2_control" default="true"/>

  <!-- ==================================================================== -->
  <!-- 2A. GAZEBO CLASSIC (Gazebo 11) PLUGINS (Only when NOT Ignition)      -->
  <!-- ==================================================================== -->
  <xacro:unless value="$(arg is_ignition)">

    <xacro:if value="$(arg use_ros2_control)">
      <!-- Option B - ros2_control System Plugin (Default for production and bringup) -->
      <gazebo>
        <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
          <parameters>$(find mobile_robot_controller)/config/ros2_robot_controller_params.yaml</parameters>
          <ros>
            <remapping>/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel</remapping>
            <remapping>/diff_drive_controller/odom:=/odom</remapping>
          </ros>
        </plugin>
      </gazebo>
    </xacro:if>

    <xacro:unless value="$(arg use_ros2_control)">
      <!-- Option A - Gazebo Classic Plugins (For Custom C++ and Python Kinematics Nodes) -->
      <gazebo>
        <plugin name="four_wheel_diff_drive" filename="libgazebo_ros_diff_drive.so">
          <ros>
            <namespace>/</namespace>
            <remapping>/cmd_vel:=/cmd_vel</remapping>
            <remapping>/odom:=/odom</remapping>
          </ros>

          <update_rate>50.0</update_rate>

          <!-- 2 Wheel Pairs for 4-wheel robot -->
          <num_wheel_pairs>2</num_wheel_pairs>

          <!-- Left Wheels -->
          <left_joint>front_left_wheel_joint</left_joint>
          <left_joint>rear_left_wheel_joint</left_joint>

          <!-- Right Wheels -->
          <right_joint>front_right_wheel_joint</right_joint>
          <right_joint>rear_right_wheel_joint</right_joint>

          <!-- Kinematics -->
          <wheel_separation>0.34</wheel_separation>
          <wheel_diameter>0.10</wheel_diameter>

          <!-- Limits -->
          <max_wheel_torque>20.0</max_wheel_torque>
          <max_wheel_acceleration>2.0</max_wheel_acceleration>

          <!-- Output / Odometry (Handled by diff_drive_controller) -->
          <publish_odom>false</publish_odom>
          <publish_odom_tf>false</publish_odom_tf>
          <publish_wheel_tf>false</publish_wheel_tf>

          <odometry_frame>odom</odometry_frame>
          <robot_base_frame>base_footprint</robot_base_frame>
        </plugin>
      </gazebo>

      <!-- Gazebo Joint State Publisher (Publishes /joint_states for wheels in Option A) -->
      <gazebo>
        <plugin name="four_wheel_joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
          <ros>
            <namespace>/</namespace>
            <remapping>~/out:=/joint_states</remapping>
          </ros>
          <update_rate>50.0</update_rate>
          <joint_name>front_left_wheel_joint</joint_name>
          <joint_name>front_right_wheel_joint</joint_name>
          <joint_name>rear_left_wheel_joint</joint_name>
          <joint_name>rear_right_wheel_joint</joint_name>
        </plugin>
      </gazebo>
    </xacro:unless>

    <!-- Gazebo Classic 2D LiDAR Ray Sensor Plugin -->
    <gazebo reference="lidar_link">
      <sensor name="lidar_sensor" type="ray">
        <pose>0 0 0.044 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>10.0</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <remapping>~/out:=/scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>lidar_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>

    <!-- Gazebo Classic Front Camera Sensor Plugin -->
    <gazebo reference="camera_link">
      <sensor name="front_camera" type="camera">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>30.0</update_rate>
        <camera>
          <horizontal_fov>1.089</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>8.0</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <remapping>~/image_raw:=/camera/image_raw</remapping>
            <remapping>~/camera_info:=/camera/camera_info</remapping>
          </ros>
          <camera_name>camera</camera_name>
          <frame_name>camera_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>

    <!-- Gazebo Classic IMU Sensor Plugin -->
    <gazebo reference="imu_link">
      <material>Gazebo/Green</material>
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>50.0</update_rate>
        <visualize>false</visualize>
        <topic>/imu/data</topic>
        <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
          <ros>
            <namespace>/</namespace>
            <remapping>~/out:=/imu/data</remapping>
          </ros>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
          <frame_name>imu_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>

  </xacro:unless>

  <!-- ==================================================================== -->
  <!-- 2B. IGNITION GAZEBO (ign) & MODERN GAZEBO (gz) SYSTEMS & PLUGINS     -->
  <!-- ==================================================================== -->
  <xacro:if value="$(arg is_ignition)">
    <gazebo>
      <xacro:if value="$(arg use_ros2_control)">
        <!-- Option B - ros2_control System Plugin for Ignition / Modern Gazebo -->
        <plugin filename="libign_ros2_control-system.so" name="ign_ros2_control::IgnitionROS2ControlPlugin">
          <parameters>$(find mobile_robot_controller)/config/ros2_robot_controller_params.yaml</parameters>
          <ros>
            <remapping>/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel</remapping>
            <remapping>/diff_drive_controller/odom:=/odom</remapping>
          </ros>
        </plugin>
      </xacro:if>

      <xacro:unless value="$(arg use_ros2_control)">
        <!-- Option A - Diff Drive System for Ignition / Modern Gazebo -->
        <plugin filename="libignition-gazebo-diff-drive-system.so" name="ignition::gazebo::systems::DiffDrive">
          <left_joint>front_left_wheel_joint</left_joint>
          <left_joint>rear_left_wheel_joint</left_joint>
          <right_joint>front_right_wheel_joint</right_joint>
          <right_joint>rear_right_wheel_joint</right_joint>
          <wheel_separation>0.34</wheel_separation>
          <wheel_radius>0.05</wheel_radius>
          <odom_publish_frequency>50</odom_publish_frequency>
          <topic>/cmd_vel</topic>
          <odom_topic>/odom</odom_topic>
          <tf_topic>/tf</tf_topic>
          <frame_id>odom</frame_id>
          <child_frame_id>base_footprint</child_frame_id>
        </plugin>
      </xacro:unless>

      <!-- Joint State Publisher for Ignition / Modern Gazebo -->
      <plugin filename="libignition-gazebo-joint-state-publisher-system.so" name="ignition::gazebo::systems::JointStatePublisher">
        <topic>/joint_states</topic>
      </plugin>

      <!-- Sensors System for Ignition / Modern Gazebo -->
      <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
        <render_engine>ogre2</render_engine>
      </plugin>

      <!-- IMU System for Ignition / Modern Gazebo -->
      <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu"/>
    </gazebo>

    <!-- Ignition / Modern Gz Sensors -->
    <gazebo reference="lidar_link">
      <sensor name="lidar_sensor_ign" type="gpu_lidar">
        <pose>0 0 0.044 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>10.0</update_rate>
        <topic>/scan</topic>
        <gz_frame_id>lidar_link</gz_frame_id>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
      </sensor>
    </gazebo>

    <gazebo reference="camera_link">
      <sensor name="front_camera_ign" type="camera">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>30.0</update_rate>
        <topic>/camera/image_raw</topic>
        <gz_frame_id>camera_link</gz_frame_id>
        <camera>
          <horizontal_fov>1.089</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>8.0</far>
          </clip>
        </camera>
      </sensor>
    </gazebo>
  </xacro:if>

</robot>
```

#### Step 2.0.5: Update `display.launch.py` with `simulation.rviz` (Fixed Frame: `odom`)
Now that we are implementing motor controllers and broadcasting the `odom -> base_footprint` dynamic transform, update `launch/display.launch.py` so that simply running `ros2 launch mobile_robot_description display.launch.py` automatically loads `simulation.rviz` with **`Fixed Frame: odom`**!

```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_description/launch/display.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def is_simulation_running():
    """Detect if Gazebo Classic or Ignition/Gz Sim is currently running."""
    for proc_pattern in ['gzserver', 'gzclient', 'ign gazebo', 'gz sim', 'ign_gazebo']:
        try:
            res = subprocess.run(['pgrep', '-f', proc_pattern], capture_output=True)
            if res.returncode == 0:
                return True
        except Exception:
            pass
    return False


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mobile_robot_description')
    
    gui_arg_val = LaunchConfiguration('gui').perform(context).lower()
    use_sim_time_arg_val = LaunchConfiguration('use_sim_time').perform(context).lower()
    run_rsp_arg_val = LaunchConfiguration('run_rsp').perform(context).lower()
    model_path = LaunchConfiguration('model').perform(context)
    sim_active = is_simulation_running()

    # 1. Determine use_sim_time FIRST
    if use_sim_time_arg_val in ['true', '1']:
        use_sim_time = True
    elif use_sim_time_arg_val in ['false', '0']:
        use_sim_time = False
    else:  # 'auto'
        use_sim_time = sim_active

    # 2. Determine gui (joint_state_publisher_gui)
    if gui_arg_val in ['true', '1']:
        enable_gui = True
    elif gui_arg_val in ['false', '0']:
        enable_gui = False
    else:  # 'auto'
        enable_gui = not sim_active

    # 3. Determine run_rsp (robot_state_publisher)
    if run_rsp_arg_val in ['true', '1']:
        enable_rsp = True
    elif run_rsp_arg_val in ['false', '0']:
        enable_rsp = False
    else:  # 'auto'
        enable_rsp = not sim_active

    # 4. Determine rviz config:
    # Standalone mode: display.rviz (Fixed Frame: base_footprint)
    # Simulation mode: simulation.rviz (Fixed Frame: odom)
    rviz_config_arg_val = LaunchConfiguration('rviz_config').perform(context)
    if rviz_config_arg_val.lower() in ['auto', 'default', '']:
        if sim_active:
            rviz_config_path = os.path.join(pkg_share, 'rviz', 'simulation.rviz')
        else:
            rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    elif rviz_config_arg_val.lower() in ['display', 'display.rviz', 'standalone']:
        rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    elif rviz_config_arg_val.lower() in ['sim', 'simulation', 'simulation.rviz']:
        rviz_config_path = os.path.join(pkg_share, 'rviz', 'simulation.rviz')
    else:
        rviz_config_path = rviz_config_arg_val

    # Process Xacro
    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str
    )

    nodes_to_launch = []

    # 1. Robot State Publisher (only if needed / not provided by gazebo)
    if enable_rsp:
        nodes_to_launch.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            )
        )

    # 2. Joint State Publisher GUI or fallback Joint State Publisher
    if enable_gui:
        nodes_to_launch.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description
                }]
            )
        )
    elif not sim_active:
        nodes_to_launch.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description
                }]
            )
        )

    # 3. RViz2 Node
    nodes_to_launch.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        )
    )

    return nodes_to_launch


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_description')
    
    default_model_path = os.path.join(pkg_share, 'urdf', 'mobile_robot.urdf.xacro')
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='auto',
        description='Enable joint_state_publisher_gui sliders: auto (true if standalone), true, or false'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='auto',
        description='Use simulation (Gazebo) clock: auto, true, or false'
    )
    
    run_rsp_arg = DeclareLaunchArgument(
        name='run_rsp',
        default_value='auto',
        description='Run robot_state_publisher: auto (false if Gazebo running), true, or false'
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf.xacro file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value='auto',
        description='Path to rviz config file: auto, display, or simulation'
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        run_rsp_arg,
        model_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])
```

#### Step 2.0.6: Rebuild `mobile_robot_description`
```bash
cd ~/ros2_mobile_robot_ws
colcon build --packages-select mobile_robot_description --symlink-install
source install/setup.bash
```

---

### 🛠️ Selecting Your Milestone 2 Track

Depending on your workshop curriculum or project architecture, this manual provides **three distinct, step-by-step options**:
* **🌟 OPTION 1: Dual Mode Controller Stack (Both Pure Coding & ROS 2 Control Coexisting):** The complete unified stack pre-configured in this repository. You build both the custom kinematics C++/Python nodes AND configure the upstream `ros2_control` framework side-by-side in one package (`mobile_robot_controller`).
* **💻 OPTION 2: Standalone Pure Coding Track (Kinematics & Odometry from First Principles):** For students who want to focus exclusively on algorithmic robotics mathematics. You write the differential drive forward/inverse kinematics and Euler dead-reckoning odometry nodes from scratch in C++ and Python (without `ros2_control`).
* **⚙️ OPTION 3: Pure ROS 2 Controller Manager Track (`ros2_control` Framework Only):** For students who want to jump straight to industry production standards. You configure `controller_manager`, `diff_drive_controller/DiffDriveController`, and `joint_state_broadcaster` using official upstream binaries.

---

### 📊 Option Comparison Matrix

| Feature / Architecture | **Option 1: Dual Mode Stack** | **Option 2: Standalone Pure Code** | **Option 3: Pure `ros2_control`** |
| :--- | :--- | :--- | :--- |
| **Primary Pedagogical Goal** | Master both algorithms & production frameworks | First-principles kinematics math & Euler dead-reckoning | Production robotics control architecture |
| **Node Implementation** | Both Custom Nodes + Upstream Spawner | Custom C++ `diff_drive_controller.cpp` & Python `diff_drive_controller.py` | Upstream `controller_manager` + `spawner` CLI |
| **C++ Header File** | `include/mobile_robot_controller/diff_drive_controller.hpp` | `include/mobile_robot_controller/diff_drive_controller.hpp` | Provided by official ROS 2 Control packages |
| **Configuration Files** | Both `config/controller_params.yaml` & `config/ros2_robot_controller_params.yaml` | `config/controller_params.yaml` | `config/ros2_robot_controller_params.yaml` |
| **Launch Files** | Both `launch/controller.launch.py` & `launch/ros2_controller.launch.py` | `launch/controller.launch.py` | `launch/ros2_controller.launch.py` |
| **Command Topics** | `/cmd_vel` | `/cmd_vel` | `/cmd_vel` (remapped from `~/cmd_vel_unstamped`) |
| **Odometry Output** | `/odom` | `/odom` + dynamic TF (`odom -> base_footprint`) | `/odom` (remapped from `~/odom`) + dynamic TF |

---

### 📐 Differential Drive Kinematics & Odometry Mathematical Foundation

```
                      Front of Robot (+X)
                              ▲
                              │
          Left Wheels         │         Right Wheels
          ┌─────────┐         │         ┌─────────┐
          │  FL (ω) │         │         │  FR (ω) │
          └────┬────┘         │         └────┬────┘
               │              │              │
               │◄─────────────┼─────────────►│
               │              │ Track Width L│
               │              │   (=0.34 m)  │
          ┌────┴────┐         │         ┌────┴────┐
          │  RL (ω) │         │         │  RR (ω) │
          └─────────┘         │         └─────────┘
                              │
                         Wheel Radius r (=0.05 m)
```

#### 1. Inverse Kinematics (Body Twist $\to$ Wheel Angular Velocities)
Given body linear velocity $v$ (m/s) and angular velocity $\omega$ (rad/s), with wheel radius $r = 0.05\text{m}$ and track width (wheel separation) $L = 0.34\text{m}$:
$$\omega_L = \frac{v - \omega \cdot \frac{L}{2}}{r}, \quad \omega_R = \frac{v + \omega \cdot \frac{L}{2}}{r}$$

* In a 4-wheel skid-steer differential platform, the two wheels on each side share identical velocity setpoints:
  $$\omega_{FL} = \omega_{RL} = \omega_L, \quad \omega_{FR} = \omega_{RR} = \omega_R$$

#### 2. Forward Kinematics (Wheel Displacements $\to$ Odometry Dead-Reckoning)
From quadrature encoder tick deltas ($\Delta \text{ticks}_L, \Delta \text{ticks}_R$) with Counts Per Revolution $CPR = 330$:
$$\Delta s_L = \frac{2\pi r \cdot \Delta \text{ticks}_L}{CPR}, \quad \Delta s_R = \frac{2\pi r \cdot \Delta \text{ticks}_R}{CPR}$$
$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \quad \Delta \theta = \frac{\Delta s_R - \Delta s_L}{L}$$

Using 2nd-order Runge-Kutta / Midpoint integration to minimize dead-reckoning accumulation error:
$$x_{k+1} = x_k + \Delta s \cdot \cos\left(\theta_k + \frac{\Delta \theta}{2}\right)$$
$$y_{k+1} = y_k + \Delta s \cdot \sin\left(\theta_k + \frac{\Delta \theta}{2}\right)$$
$$\theta_{k+1} = \theta_k + \Delta \theta$$

#### 3. Dynamic Coordinate Frame Transformation (TF Tree)
The odometry estimator broadcasts a dynamic transform from the fixed world odometry frame (`odom`) to the robot moving base frame (`base_footprint`):
$$\mathbf{T}_{\text{odom}}^{\text{base\_footprint}} = \begin{bmatrix} \cos\theta & -\sin\theta & 0 & x \\ \sin\theta & \cos\theta & 0 & y \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

---

### 🌟 OPTION 1: Dual Mode Controller Stack (Both Pure Coding & ROS 2 Control Coexisting)

*This is the master setup present in this repository. Follow this track if you want both custom kinematics coding and upstream `ros2_control` integrated side-by-side in one package.*

#### Unified Directory Tree (`mobile_robot_controller`)
```text
mobile_robot_controller/
├── CMakeLists.txt                         # Unified build instructions
├── package.xml                            # Unified dependency manifest
├── config/
│   ├── controller_params.yaml             # Kinematics physical parameters
│   └── ros2_robot_controller_params.yaml  # ros2_control manager & diff_drive
├── include/
│   └── mobile_robot_controller/
│       └── diff_drive_controller.hpp      # C++ Kinematics Header
├── launch/
│   ├── controller.launch.py               # Launches C++ or Python kinematics node
│   └── ros2_controller.launch.py          # Spawns ros2_control controllers
├── scripts/
│   └── diff_drive_controller.py           # Python kinematics node
└── src/
    └── diff_drive_controller.cpp          # C++ kinematics node
```

#### Step 2.1-A: Create Package with `ros2 pkg create`
```bash
cd ~/ros2_mobile_robot_ws/src
ros2 pkg create --build-type ament_cmake mobile_robot_controller
```

#### Step 2.2-A: Create Subdirectories
```bash
cd ~/ros2_mobile_robot_ws/src/mobile_robot_controller
mkdir -p include/mobile_robot_controller src scripts config launch
```

#### Step 2.3-A: Create Physical Kinematics Parameters (`config/controller_params.yaml`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/controller_params.yaml
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/controller_params.yaml
```
*Paste and save:*
```yaml
/**:
  ros__parameters:
    wheel_radius: 0.05         # Wheel radius in meters (5 cm)
    wheel_separation: 0.34     # Distance between left and right wheels in meters (34 cm)
    publish_rate: 50.0         # Controller update frequency in Hz (synced to 50 Hz Gazebo physics)
    cmd_vel_timeout: 0.5       # Auto-stop watchdog timeout in seconds
    odom_frame_id: "odom"      # Odometry parent frame
    base_frame_id: "base_footprint" # Robot base footprint frame
    publish_joint_states: false # Handled cleanly by joint_state_broadcaster from physics simulation
```

#### Step 2.4-A: Create C++ Controller Header (`include/mobile_robot_controller/diff_drive_controller.hpp`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/include/mobile_robot_controller/diff_drive_controller.hpp
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/include/mobile_robot_controller/diff_drive_controller.hpp
```
*Paste and save:*
```cpp
/**
 * ============================================================================
 * Mobile Robot Differential Drive Controller Header (C++)
 * ----------------------------------------------------------------------------
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 *
 * Designed for ROS 2 Mobile Robotics Workshop.
 * ============================================================================
 */

#ifndef MOBILE_ROBOT_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define MOBILE_ROBOT_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

class DiffDriveControllerCpp : public rclcpp::Node
{
public:
  DiffDriveControllerCpp();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void updateLoop();

  // Physical Parameters
  double r_;
  double L_;
  double rate_;
  std::string odom_frame_;
  std::string base_frame_;
  int cpr_;
  bool publish_joint_states_;
  double cmd_vel_timeout_;

  // Kinematic State
  double cmd_linear_x_;
  double cmd_angular_z_;
  double wheel_speed_left_;
  double wheel_speed_right_;
  double left_wheel_pos_;
  double right_wheel_pos_;
  double x_;
  double y_;
  double theta_;
  rclcpp::Time last_time_;
  rclcpp::Time last_cmd_vel_time_;
  bool has_cmd_vel_;
  int stop_count_;

  // Encoder Tracking
  bool has_encoder_data_;
  bool has_prev_ticks_;
  int32_t prev_left_ticks_;
  int32_t prev_right_ticks_;
  double delta_s_left_;
  double delta_s_right_;

  // ROS 2 Interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_gazebo_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // MOBILE_ROBOT_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
```

#### Step 2.5-A: Create C++ Kinematics Implementation (`src/diff_drive_controller.cpp`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/src/diff_drive_controller.cpp
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/src/diff_drive_controller.cpp
```
*Paste and save:*
```cpp
/**
 * ============================================================================
 * Mobile Robot Differential Drive Controller (C++)
 * ----------------------------------------------------------------------------
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 *
 * Designed for ROS 2 Mobile Robotics Workshop.
 * This node performs:
 *  1. Inverse Kinematics: Translates /cmd_vel into Left/Right wheel angular speeds.
 *  2. Forward Kinematics (Dead-Reckoning Odometry): Computes robot pose (x, y, theta).
 *  3. Joint State Publishing: Publishes wheel positions to animate 4 wheels in RViz.
 *  4. TF Broadcasting: Broadcasts dynamic transformation 'odom' -> 'base_footprint'.
 *  5. Hardware Command Publishing: Publishes wheel speeds for Arduino firmware.
 * ============================================================================
 */

#include "mobile_robot_controller/diff_drive_controller.hpp"

using namespace std::chrono_literals;

DiffDriveControllerCpp::DiffDriveControllerCpp()
: Node("diff_drive_controller_cpp")
{
  // -------------------------------------------------------------------------
  // 1. Declare and Read ROS 2 Parameters
  // -------------------------------------------------------------------------
  this->declare_parameter<double>("wheel_radius", 0.05);         // meters
  this->declare_parameter<double>("wheel_separation", 0.34);     // meters (track width)
  this->declare_parameter<double>("publish_rate", 50.0);         // Hz
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  this->declare_parameter<int>("encoder_cpr", 330);
  this->declare_parameter<bool>("publish_joint_states", true);
  this->declare_parameter<double>("cmd_vel_timeout", 0.5);

  r_ = this->get_parameter("wheel_radius").as_double();
  L_ = this->get_parameter("wheel_separation").as_double();
  rate_ = this->get_parameter("publish_rate").as_double();
  odom_frame_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_ = this->get_parameter("base_frame_id").as_string();
  cpr_ = this->get_parameter("encoder_cpr").as_int();
  publish_joint_states_ = this->get_parameter("publish_joint_states").as_bool();
  cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();

  // -------------------------------------------------------------------------
  // 2. Initialize Variables
  // -------------------------------------------------------------------------
  cmd_linear_x_ = 0.0;
  cmd_angular_z_ = 0.0;
  wheel_speed_left_ = 0.0;
  wheel_speed_right_ = 0.0;
  left_wheel_pos_ = 0.0;
  right_wheel_pos_ = 0.0;
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  last_time_ = this->now();
  last_cmd_vel_time_ = this->now();
  has_cmd_vel_ = false;
  stop_count_ = 0;

  has_encoder_data_ = false;
  has_prev_ticks_ = false;
  prev_left_ticks_ = 0;
  prev_right_ticks_ = 0;
  delta_s_left_ = 0.0;
  delta_s_right_ = 0.0;

  // -------------------------------------------------------------------------
  // 3. Subscribers, Publishers & TF Broadcaster
  // -------------------------------------------------------------------------
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&DiffDriveControllerCpp::cmdVelCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/wheel_encoder_ticks", 10,
    std::bind(&DiffDriveControllerCpp::encoderCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed_commands", 10);
  wheel_cmd_gazebo_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_velocity_controller/commands", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Periodic Update Timer (using simulation-aware ROS clock timer for smooth TF sync)
  auto timer_period = rclcpp::Duration::from_seconds(1.0 / rate_);
  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    timer_period,
    std::bind(&DiffDriveControllerCpp::updateLoop, this));

  RCLCPP_INFO(this->get_logger(), "============================================================");
  RCLCPP_INFO(this->get_logger(), "🚀 C++ Diff Drive Controller Initialized");
  RCLCPP_INFO(this->get_logger(), "   Wheel Radius:     %.3f m", r_);
  RCLCPP_INFO(this->get_logger(), "   Wheel Separation: %.3f m", L_);
  RCLCPP_INFO(this->get_logger(), "   Update Frequency: %.1f Hz", rate_);
  RCLCPP_INFO(this->get_logger(), "============================================================");
}

void DiffDriveControllerCpp::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (std::abs(msg->linear.x) < 1e-4 && std::abs(msg->angular.z) < 1e-4) {
    if (!has_cmd_vel_) {
      return;
    }
    cmd_linear_x_ = 0.0;
    cmd_angular_z_ = 0.0;
    wheel_speed_left_ = 0.0;
    wheel_speed_right_ = 0.0;
    has_cmd_vel_ = false;
    return;
  }
  last_cmd_vel_time_ = this->now();
  has_cmd_vel_ = true;

  cmd_linear_x_ = msg->linear.x;
  cmd_angular_z_ = msg->angular.z;

  // -------------------------------------------------------------------------
  // INVERSE KINEMATICS:
  // V_left  = V - (W * L / 2)
  // V_right = V + (W * L / 2)
  // W_left  = V_left / R
  // W_right = V_right / R
  // -------------------------------------------------------------------------
  double v_left = cmd_linear_x_ - (cmd_angular_z_ * L_ / 2.0);
  double v_right = cmd_linear_x_ + (cmd_angular_z_ * L_ / 2.0);

  wheel_speed_left_ = v_left / r_;
  wheel_speed_right_ = v_right / r_;
}

void DiffDriveControllerCpp::encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    return;
  }
  int32_t left_ticks = msg->data[0];
  int32_t right_ticks = msg->data[1];

  if (has_prev_ticks_) {
    int32_t d_left = left_ticks - prev_left_ticks_;
    int32_t d_right = right_ticks - prev_right_ticks_;

    double meters_per_tick = (2.0 * M_PI * r_) / static_cast<double>(cpr_);
    delta_s_left_ = d_left * meters_per_tick;
    delta_s_right_ = d_right * meters_per_tick;
    has_encoder_data_ = true;
  }

  prev_left_ticks_ = left_ticks;
  prev_right_ticks_ = right_ticks;
  has_prev_ticks_ = true;
}

void DiffDriveControllerCpp::updateLoop()
{
  rclcpp::Time current_time = this->now();

  // -------------------------------------------------------------------------
  // Safety Watchdog: If /cmd_vel publisher stops, auto-stop robot after 0.5s
  // -------------------------------------------------------------------------
  if (has_cmd_vel_ && (current_time - last_cmd_vel_time_).seconds() > cmd_vel_timeout_) {
    cmd_linear_x_ = 0.0;
    cmd_angular_z_ = 0.0;
    wheel_speed_left_ = 0.0;
    wheel_speed_right_ = 0.0;
    has_cmd_vel_ = false;
    stop_count_ = 25;  // Actively send zero twists for 0.5s at 50Hz to halt simulation
  }

  if (stop_count_ > 0) {
    stop_count_--;
    geometry_msgs::msg::Twist stop_twist;
    cmd_vel_pub_->publish(stop_twist);

    std_msgs::msg::Float64MultiArray stop_wheel_cmds;
    stop_wheel_cmds.data = {0.0, 0.0, 0.0, 0.0};
    wheel_cmd_gazebo_pub_->publish(stop_wheel_cmds);
  }

  double dt = (current_time - last_time_).seconds();
  if (dt <= 0.0) {
    return;
  }
  if (dt > 1.0) {
    last_time_ = current_time;
    return;
  }
  last_time_ = current_time;

  // -------------------------------------------------------------------------
  // FORWARD KINEMATICS & POSE INTEGRATION:
  // -------------------------------------------------------------------------
  double delta_s = 0.0;
  double delta_theta = 0.0;
  double v_robot = 0.0;
  double w_robot = 0.0;

  if (has_encoder_data_ && (std::abs(delta_s_left_) > 1e-6 || std::abs(delta_s_right_) > 1e-6)) {
    delta_s = (delta_s_right_ + delta_s_left_) / 2.0;
    delta_theta = (delta_s_right_ - delta_s_left_) / L_;
    delta_s_left_ = 0.0;
    delta_s_right_ = 0.0;

    v_robot = delta_s / dt;
    w_robot = delta_theta / dt;
  } else {
    double v_left_linear = wheel_speed_left_ * r_;
    double v_right_linear = wheel_speed_right_ * r_;

    v_robot = (v_right_linear + v_left_linear) / 2.0;
    w_robot = (v_right_linear - v_left_linear) / L_;

    delta_s = v_robot * dt;
    delta_theta = w_robot * dt;
  }

  // Euler Integration (Midpoint Runge-Kutta 2nd Order)
  x_ += delta_s * std::cos(theta_ + delta_theta / 2.0);
  y_ += delta_s * std::sin(theta_ + delta_theta / 2.0);
  theta_ += delta_theta;

  // Normalize theta to [-pi, pi]
  theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

  // Integrate wheel rotations for RViz animation
  left_wheel_pos_ += wheel_speed_left_ * dt;
  right_wheel_pos_ += wheel_speed_right_ * dt;

  // Quaternion from yaw
  double qz = std::sin(theta_ / 2.0);
  double qw = std::cos(theta_ / 2.0);

  // -------------------------------------------------------------------------
  // 1. Publish TF: 'odom' -> 'base_footprint'
  // -------------------------------------------------------------------------
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = current_time;
  t.header.frame_id = odom_frame_;
  t.child_frame_id = base_frame_;

  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;

  tf_broadcaster_->sendTransform(t);

  // -------------------------------------------------------------------------
  // 2. Publish /odom Topic
  // -------------------------------------------------------------------------
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;

  // Set covariance to prevent oversized RViz covariance bubbles
  odom.pose.covariance = {
    0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
    0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
    0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
    0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
    0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
    0.0,   0.0,   0.0, 0.0, 0.0, 0.01
  };

  odom.twist.twist.linear.x = v_robot;
  odom.twist.twist.angular.z = w_robot;
  odom.twist.covariance = odom.pose.covariance;

  odom_pub_->publish(odom);

  // -------------------------------------------------------------------------
  // 3. Publish /joint_states Topic (all 4 wheels)
  // -------------------------------------------------------------------------
  sensor_msgs::msg::JointState js;
  js.header.stamp = current_time;
  js.name = {
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint"
  };
  js.position = {left_wheel_pos_, right_wheel_pos_, left_wheel_pos_, right_wheel_pos_};
  js.velocity = {wheel_speed_left_, wheel_speed_right_, wheel_speed_left_, wheel_speed_right_};

  if (publish_joint_states_) {
    joint_state_pub_->publish(js);
  }

  // -------------------------------------------------------------------------
  // 4. Publish /wheel_speed_commands (for Arduino firmware)
  // -------------------------------------------------------------------------
  std_msgs::msg::Float32MultiArray wheel_cmds;
  wheel_cmds.data = {static_cast<float>(wheel_speed_left_), static_cast<float>(wheel_speed_right_)};
  wheel_cmd_pub_->publish(wheel_cmds);

  // 5. Publish to Gazebo actuator controller (/joint_group_velocity_controller/commands)
  std_msgs::msg::Float64MultiArray gazebo_wheel_cmds;
  gazebo_wheel_cmds.data = {wheel_speed_left_, wheel_speed_right_, wheel_speed_left_, wheel_speed_right_};
  wheel_cmd_gazebo_pub_->publish(gazebo_wheel_cmds);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiffDriveControllerCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

#### Step 2.6-A: Create Python Kinematics Implementation (`scripts/diff_drive_controller.py`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/scripts/diff_drive_controller.py
chmod +x ~/ros2_mobile_robot_ws/src/mobile_robot_controller/scripts/diff_drive_controller.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/scripts/diff_drive_controller.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Mobile Robot Differential Drive Controller (Python)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Designed for ROS 2 Mobile Robotics Workshop.
This node performs:
 1. Inverse Kinematics: Translates /cmd_vel (Twist) into Left/Right wheel angular speeds.
 2. Forward Kinematics (Dead-Reckoning Odometry): Computes robot pose (x, y, theta).
 3. Joint State Publishing: Publishes wheel rotations to animate all 4 wheels in RViz.
 4. TF Broadcasting: Broadcasts the dynamic transformation 'odom' -> 'base_footprint'.
 5. Hardware Command Publishing: Publishes wheel speeds for the Arduino firmware.
================================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from tf2_ros import TransformBroadcaster


class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # ---------------------------------------------------------------------
        # 1. Declare and Read ROS 2 Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter('wheel_radius', 0.05)         # meters
        self.declare_parameter('wheel_separation', 0.34)     # meters (track width)
        self.declare_parameter('publish_rate', 30.0)         # Hz
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('encoder_cpr', 330)           # Counts per rev
        self.declare_parameter('publish_joint_states', True)

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.cpr = self.get_parameter('encoder_cpr').get_parameter_value().integer_value
        self.publish_joint_states = self.get_parameter('publish_joint_states').get_parameter_value().bool_value
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value

        # Encoder tracking
        self.has_encoder_data = False
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.delta_s_left = 0.0
        self.delta_s_right = 0.0

        # ---------------------------------------------------------------------
        # 2. Internal Kinematic State Variables
        # ---------------------------------------------------------------------
        # Target velocities from /cmd_vel
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # Wheel angular velocities (rad/s)
        self.wheel_speed_left = 0.0
        self.wheel_speed_right = 0.0

        # Accumulated wheel angular positions (radians, for joint_states)
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Integrated Robot Pose in Odom frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timestamp tracking for numerical integration (dt)
        self.last_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()
        self.has_cmd_vel = False
        self.stop_count = 0

        # ---------------------------------------------------------------------
        # 3. Subscribers and Publishers
        # ---------------------------------------------------------------------
        from std_msgs.msg import Int32MultiArray

        # Subscribe to velocity commands (from keyboard teleop or navigation)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher to /cmd_vel for safety watchdog auto-stop in Gazebo
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Gazebo actuator velocity command publisher
        self.wheel_cmd_gazebo_pub = self.create_publisher(Float64MultiArray, '/joint_group_velocity_controller/commands', 10)

        # Subscribe to wheel encoder ticks for closed-loop odometry
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/wheel_encoder_ticks',
            self.encoder_ticks_callback,
            10
        )

        # Publish Odometry for navigation algorithms (SLAM / Nav2)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Publish Joint States so RViz rotates the 3D wheel meshes
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Publish Wheel Speed Commands for microcontroller / Arduino firmware
        self.wheel_cmd_pub = self.create_publisher(Float32MultiArray, '/wheel_speed_commands', 10)

        # TF Broadcaster for 'odom' -> 'base_footprint'
        self.tf_broadcaster = TransformBroadcaster(self)

        # Periodic Timer Loop (using simulation-aware ROS clock)
        self.timer = self.create_timer(1.0 / self.rate, self.update_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 Python Diff Drive Controller Initialized')
        self.get_logger().info(f'   Wheel Radius:     {self.r} m')
        self.get_logger().info(f'   Wheel Separation: {self.L} m')
        self.get_logger().info(f'   Update Frequency: {self.rate} Hz')
        self.get_logger().info('=' * 60)

    def cmd_vel_callback(self, msg: Twist):
        """Callback executed whenever a new /cmd_vel Twist message arrives."""
        if abs(msg.linear.x) < 1e-4 and abs(msg.angular.z) < 1e-4:
            if not self.has_cmd_vel:
                return
            self.cmd_linear_x = 0.0
            self.cmd_angular_z = 0.0
            self.wheel_speed_left = 0.0
            self.wheel_speed_right = 0.0
            self.has_cmd_vel = False
            return

        self.last_cmd_vel_time = self.get_clock().now()
        self.has_cmd_vel = True

        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

        # ---------------------------------------------------------------------
        # INVERSE KINEMATICS:
        # Given desired robot linear velocity V (m/s) and angular velocity W (rad/s):
        #   V_left  = V - (W * L / 2)
        #   V_right = V + (W * L / 2)
        # Rotational speeds (rad/s):
        #   W_left  = V_left / R
        #   W_right = V_right / R
        # ---------------------------------------------------------------------
        v_left = self.cmd_linear_x - (self.cmd_angular_z * self.L / 2.0)
        v_right = self.cmd_linear_x + (self.cmd_angular_z * self.L / 2.0)

        self.wheel_speed_left = v_left / self.r
        self.wheel_speed_right = v_right / self.r

    def encoder_ticks_callback(self, msg):
        """Processes real hardware encoder tick counts."""
        if len(msg.data) < 2:
            return

        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        if self.prev_left_ticks is not None and self.prev_right_ticks is not None:
            d_left = left_ticks - self.prev_left_ticks
            d_right = right_ticks - self.prev_right_ticks

            # Distance moved by each wheel side (meters)
            meters_per_tick = (2.0 * math.pi * self.r) / float(self.cpr)
            self.delta_s_left = d_left * meters_per_tick
            self.delta_s_right = d_right * meters_per_tick
            self.has_encoder_data = True

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

    def update_loop(self):
        """Periodic loop: updates odometry, publishes wheel commands and TF."""
        current_time = self.get_clock().now()

        # ---------------------------------------------------------------------
        # Safety Watchdog: If /cmd_vel publisher stops, auto-stop robot after 0.5s
        # ---------------------------------------------------------------------
        if self.has_cmd_vel and (current_time - self.last_cmd_vel_time).nanoseconds / 1e9 > self.cmd_vel_timeout:
            self.cmd_linear_x = 0.0
            self.cmd_angular_z = 0.0
            self.wheel_speed_left = 0.0
            self.wheel_speed_right = 0.0
            self.has_cmd_vel = False
            self.stop_count = 25  # Actively send zero twists for 0.5s at 50Hz to halt simulation

        if self.stop_count > 0:
            self.stop_count -= 1
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)

            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.wheel_cmd_gazebo_pub.publish(stop_msg)

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        if dt > 1.0:
            self.last_time = current_time
            return
        self.last_time = current_time

        # ---------------------------------------------------------------------
        # FORWARD KINEMATICS & POSE INTEGRATION:
        # If real encoder data is available, compute displacement from encoders.
        # Otherwise, fall back to velocity command dead-reckoning.
        # ---------------------------------------------------------------------
        if self.has_encoder_data and (abs(self.delta_s_left) > 1e-6 or abs(self.delta_s_right) > 1e-6):
            delta_s = (self.delta_s_right + self.delta_s_left) / 2.0
            delta_theta = (self.delta_s_right - self.delta_s_left) / self.L
            # Reset delta for next tick cycle
            self.delta_s_left = 0.0
            self.delta_s_right = 0.0

            v_robot = delta_s / dt
            w_robot = delta_theta / dt
        else:
            v_left_linear = self.wheel_speed_left * self.r
            v_right_linear = self.wheel_speed_right * self.r
            v_robot = (v_right_linear + v_left_linear) / 2.0
            w_robot = (v_right_linear - v_left_linear) / self.L

            delta_s = v_robot * dt
            delta_theta = w_robot * dt

        # Midpoint / Euler numerical integration
        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Integrate wheel angular positions for RViz wheel spinning animation
        self.left_wheel_pos += self.wheel_speed_left * dt
        self.right_wheel_pos += self.wheel_speed_right * dt

        # Convert Euler yaw (theta) to Quaternion: (qx, qy, qz, qw)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # ---------------------------------------------------------------------
        # 1. Publish TF: 'odom' -> 'base_footprint'
        # ---------------------------------------------------------------------
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # ---------------------------------------------------------------------
        # 2. Publish /odom Topic
        # ---------------------------------------------------------------------
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose in odom frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Accurate small covariance to prevent huge RViz covariance bubbles
        odom_msg.pose.covariance = [
            0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
            0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
            0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
            0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
            0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
            0.0,   0.0,   0.0, 0.0, 0.0, 0.01
        ]

        # Velocity in base_footprint frame
        odom_msg.twist.twist.linear.x = v_robot
        odom_msg.twist.twist.angular.z = w_robot
        odom_msg.twist.covariance = odom_msg.pose.covariance

        self.odom_pub.publish(odom_msg)

        # ---------------------------------------------------------------------
        # 3. Publish /joint_states Topic (all 4 wheels)
        # ---------------------------------------------------------------------
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        joint_state.position = [
            self.left_wheel_pos,
            self.right_wheel_pos,
            self.left_wheel_pos,
            self.right_wheel_pos
        ]
        joint_state.velocity = [
            self.wheel_speed_left,
            self.wheel_speed_right,
            self.wheel_speed_left,
            self.wheel_speed_right
        ]
        if self.publish_joint_states:
            self.joint_state_pub.publish(joint_state)

        # ---------------------------------------------------------------------
        # 4. Publish /wheel_speed_commands (for Arduino firmware)
        # ---------------------------------------------------------------------
        wheel_cmds = Float32MultiArray()
        # [left_rad_s, right_rad_s]
        wheel_cmds.data = [float(self.wheel_speed_left), float(self.wheel_speed_right)]
        self.wheel_cmd_pub.publish(wheel_cmds)

        # 5. Publish to Gazebo actuator controller (/joint_group_velocity_controller/commands)
        gazebo_cmds = Float64MultiArray()
        gazebo_cmds.data = [
            float(self.wheel_speed_left),
            float(self.wheel_speed_right),
            float(self.wheel_speed_left),
            float(self.wheel_speed_right)
        ]
        self.wheel_cmd_gazebo_pub.publish(gazebo_cmds)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
```

#### Step 2.7-A: Create Custom Kinematics Launch File (`launch/controller.launch.py`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/controller.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/controller.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_controller')
    default_params = os.path.join(pkg_share, 'config', 'controller_params.yaml')

    # Launch Arguments
    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Choose C++ controller (true) or Python controller (false)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true (default: true)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to controller_params.yaml'
    )

    # 1. C++ Controller Node
    cpp_controller_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller_cpp',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('use_cpp'))
    )

    # 2. Python Controller Node
    py_controller_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller_py',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=UnlessCondition(LaunchConfiguration('use_cpp'))
    )

    # 3. Spawners for ros2_control hardware interface
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--unload-on-kill',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    joint_group_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_group_velocity_controller',
            '--controller-manager',
            '/controller_manager',
            '--unload-on-kill',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_cpp_arg,
        use_sim_time_arg,
        params_file_arg,
        joint_state_broadcaster_spawner,
        joint_group_velocity_controller_spawner,
        cpp_controller_node,
        py_controller_node
    ])
```

#### Step 2.8-A: Create `ros2_control` Parameters (`config/ros2_robot_controller_params.yaml`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/ros2_robot_controller_params.yaml
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/ros2_robot_controller_params.yaml
```
*Paste and save:*
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_group_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

joint_group_velocity_controller:
  ros__parameters:
    joints:
      - front_left_wheel_joint
      - front_right_wheel_joint
      - rear_left_wheel_joint
      - rear_right_wheel_joint

diff_drive_controller:
  ros__parameters:
    use_sim_time: true
    left_wheel_names:
      - front_left_wheel_joint
      - rear_left_wheel_joint
    right_wheel_names:
      - front_right_wheel_joint
      - rear_right_wheel_joint

    wheel_separation: 0.34
    wheel_radius: 0.05

    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0

    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_footprint
    pose_covariance_diagonal: [0.001, 0.001, 1.0e-3, 1.0e-3, 1.0e-3, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 1.0e-3, 1.0e-3, 1.0e-3, 0.01]

    open_loop: false
    enable_odom_tf: true

    cmd_vel_timeout: 0.5
    use_stamped_vel: false

    # Velocity and acceleration limits
    linear:
      x:
        has_velocity_limits: true
        max_velocity: 0.3
        min_velocity: -0.3
        has_acceleration_limits: true
        max_acceleration: 1.0
        min_acceleration: -1.0
    angular:
      z:
        has_velocity_limits: true
        max_velocity: 0.6
        min_velocity: -0.6
        has_acceleration_limits: true
        max_acceleration: 2.0
        min_acceleration: -2.0
```

#### Step 2.9-A: Create Controller Spawner Launch File (`launch/ros2_controller.launch.py`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/ros2_controller.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/ros2_controller.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
ros2_control Controller Spawner Launch
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Designed for ROS 2 Mobile Robotics Workshop.
Spawns:
  1. joint_state_broadcaster
  2. diff_drive_controller
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--unload-on-kill',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--unload-on-kill',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])
```

#### Step 2.10-A: Edit Unified `package.xml` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/package.xml
```
*Paste and save:*
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mobile_robot_controller</name>
  <version>1.0.0</version>
  <description>Unified Controller Stack: Custom Kinematics &amp; ros2_control for 4-wheel mobile robot.</description>
  <maintainer email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</maintainer>
  <author email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</author>
  <license>Apache-2.0</license>

  <url type="website">https://rahul-r-joshua.github.io/portfolio/</url>
  <url type="repository">https://github.com/rahul-r-joshua</url>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Option 1: Custom Kinematics Dependencies -->
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <!-- Option 2: Upstream ros2_control Dependencies -->
  <depend>controller_manager</depend>
  <depend>diff_drive_controller</depend>
  <depend>joint_state_broadcaster</depend>
  <depend>joint_state_publisher</depend>
  <depend>robot_state_publisher</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### Step 2.11-A: Edit Unified `CMakeLists.txt` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/CMakeLists.txt
```
*Paste and save:*
```cmake
cmake_minimum_required(VERSION 3.8)
project(mobile_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 1. C++ Kinematics Node Executable
add_executable(diff_drive_controller_cpp src/diff_drive_controller.cpp)
target_include_directories(diff_drive_controller_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diff_drive_controller_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# Install C++ binary
install(TARGETS
  diff_drive_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include
)

# 2. Python Scripts (Install both original name and _py alias for launch file compatibility)
install(PROGRAMS
  scripts/diff_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME diff_drive_controller_py
)

install(PROGRAMS
  scripts/diff_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install Config and Launch directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

#### Step 2.12-A: Build Unified Package
```bash
cd ~/ros2_mobile_robot_ws
colcon build --packages-select mobile_robot_controller --symlink-install
source install/setup.bash
```

---

#### ⚡ Option 1 Verification (Terminal-by-Terminal Execution)

> [!IMPORTANT]
> **⚠️ Beginner Rule: In EVERY new terminal tab you open, always run:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_mobile_robot_ws/install/setup.bash
> ```

##### Mode 1A Verification: Testing Custom Kinematics Stack (Pure Coding)
* **Terminal 1: Launch Simulation Engine (1st: Gazebo / Ignition / Gz)**
  *Option A: Gazebo Classic (Gazebo 11):*
  ```bash
  cd ~/ros2_mobile_robot_ws
  source /opt/ros/humble/setup.bash
  source /usr/share/gazebo/setup.bash
  source install/setup.bash
  ros2 launch mobile_robot_description gazebo.launch.py
  ```
  *Option B: Ignition Gazebo (Fortress):*
  ```bash
  cd ~/ros2_mobile_robot_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch mobile_robot_description ign_gazebo.launch.py
  ```
  *Option C: Modern Gazebo (Gz Sim / Harmonic / Garden):*
  ```bash
  cd ~/ros2_mobile_robot_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch mobile_robot_description ign_gazebo.launch.py backend:=gz
  ```
* **Terminal 2: Launch Custom Kinematics Controller (2nd: Controller)**
  *Option A: Launch C++ Kinematics Controller Node (`rclcpp`):*
  ```bash
  cd ~/ros2_mobile_robot_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch mobile_robot_controller controller.launch.py use_cpp:=true
  ```
  *Option B: Launch Python Kinematics Controller Node (`rclpy`):*
  ```bash
  cd ~/ros2_mobile_robot_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch mobile_robot_controller controller.launch.py use_cpp:=false
  ```
* **Terminal 3: Launch RViz Visualization (3rd: Display / RViz)**
  ```bash
  cd ~/ros2_mobile_robot_ws
  source install/setup.bash
  ros2 launch mobile_robot_description display.launch.py
  ```
  *(Pre-loaded with **Fixed Frame: `odom`**, live Odometry path, LiDAR points, and front Camera feed).*
* **Terminal 4: Test Kinematic Motions (`ros2 topic pub`)**
  *(Tip: `--times 20` sends 20 messages at 10 Hz over 2.0 seconds and automatically returns the prompt. Alternatively, remove `--times 20` for continuous streaming and press `Ctrl+C` to stop).*
  *Test 1: Drive Forward (+0.3 m/s for 2s):*
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
  ```
  *Test 2: Drive Backward (-0.3 m/s for 2s):*
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.2}, angular: {z: 0.0}}"
  ```
  *Test 3: Left Turn In-Place (+0.5 rad/s for 2s):*
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.4}}"
  ```
  *Test 4: Right Turn In-Place (-0.5 rad/s for 2s):*
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.4}}"
  ```
  *Test 5: Forward Arc with Right Turn (+0.3 m/s, -0.3 rad/s):*
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: -0.3}}"
  ```
  *Test 6: Backward Arc with Left Turn (-0.3 m/s, +0.3 rad/s):*
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.2}, angular: {z: 0.3}}"
  ```
* **Terminal 5: Echo Odometry (`/odom`)**
  *Single-shot inspection (prints one message and exits cleanly):*
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic echo /odom --once
  ```
  *Continuous stream (press Ctrl+C to exit):*
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic echo /odom
  ```

##### Mode 1B Verification: Testing Upstream `ros2_control` Stack
* **Terminal 1: Launch Simulation Engine (1st: Gazebo / Ignition / Gz)**
  *Option A: Gazebo Classic (Gazebo 11):*
  ```bash
  cd ~/ros2_mobile_robot_ws
  source /opt/ros/humble/setup.bash
  source /usr/share/gazebo/setup.bash
  source install/setup.bash
  ros2 launch mobile_robot_description gazebo.launch.py
  ```
  *Option B: Ignition Gazebo (Fortress):*
  ```bash
  cd ~/ros2_mobile_robot_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch mobile_robot_description ign_gazebo.launch.py
  ```
  *Option C: Modern Gazebo (Gz Sim / Harmonic / Garden):*
  ```bash
  cd ~/ros2_mobile_robot_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch mobile_robot_description ign_gazebo.launch.py backend:=gz
  ```
* **Terminal 2: Spawn `ros2_control` Controllers (2nd: Controller)**
  ```bash
  cd ~/ros2_mobile_robot_ws
  source install/setup.bash
  ros2 launch mobile_robot_controller ros2_controller.launch.py
  ```
  *(Spawns `joint_state_broadcaster` and `diff_drive_controller` without waiting, since `/controller_manager` is already running in Gazebo / Ignition!)*
* **Terminal 3: Launch RViz Visualization (3rd: Display / RViz)**
  ```bash
  cd ~/ros2_mobile_robot_ws
  source install/setup.bash
  ros2 launch mobile_robot_description display.launch.py
  ```
* **Terminal 4: Publish Driving Commands (`ros2 topic pub`)**
  *Drive forward (+0.5 m/s for 2s):*
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
  ```
  *Turn in place (+1.0 rad/s for 2s):*
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.4}}"
  ```
* **Terminal 5: Echo Odometry (`/odom`)**
  *Single-shot odometry inspection:*
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic echo /odom --once
  ```
  *Continuous stream:*
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic echo /odom
  ```

---

### 💻 OPTION 2: Standalone Pure Coding Track (Kinematics & Odometry from First Principles)

*Follow this track starting immediately after Milestone 1 if you want to focus exclusively on building differential drive kinematics and dead-reckoning odometry completely from scratch in C++ and Python (without `ros2_control`).*

#### Step 2.1-B: Create Package with `ros2 pkg create`
```bash
cd ~/ros2_mobile_robot_ws/src
ros2 pkg create --build-type ament_cmake mobile_robot_controller
```

#### Step 2.2-B: Create Subdirectories
```bash
cd ~/ros2_mobile_robot_ws/src/mobile_robot_controller
mkdir -p include/mobile_robot_controller src scripts config launch
```

#### Step 2.3-B: Create Physical Kinematics Parameters (`config/controller_params.yaml`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/controller_params.yaml
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/controller_params.yaml
```
*Paste and save:*
```yaml
/**:
  ros__parameters:
    wheel_radius: 0.05         # Wheel radius in meters (5 cm)
    wheel_separation: 0.34     # Distance between left and right wheels in meters (34 cm)
    publish_rate: 50.0         # Controller update frequency in Hz (synced to 50 Hz Gazebo physics)
    cmd_vel_timeout: 0.5       # Auto-stop watchdog timeout in seconds
    odom_frame_id: "odom"      # Odometry parent frame
    base_frame_id: "base_footprint" # Robot base footprint frame
    publish_joint_states: false # Handled cleanly by joint_state_broadcaster from physics simulation
```

#### Step 2.4-B: Create C++ Controller Header (`include/mobile_robot_controller/diff_drive_controller.hpp`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/include/mobile_robot_controller/diff_drive_controller.hpp
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/include/mobile_robot_controller/diff_drive_controller.hpp
```
*Paste and save:*
```cpp
/**
 * ============================================================================
 * Mobile Robot Differential Drive Controller Header (C++)
 * ----------------------------------------------------------------------------
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 *
 * Designed for ROS 2 Mobile Robotics Workshop.
 * ============================================================================
 */

#ifndef MOBILE_ROBOT_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define MOBILE_ROBOT_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

class DiffDriveControllerCpp : public rclcpp::Node
{
public:
  DiffDriveControllerCpp();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void updateLoop();

  // Physical Parameters
  double r_;
  double L_;
  double rate_;
  std::string odom_frame_;
  std::string base_frame_;
  int cpr_;
  bool publish_joint_states_;
  double cmd_vel_timeout_;

  // Kinematic State
  double cmd_linear_x_;
  double cmd_angular_z_;
  double wheel_speed_left_;
  double wheel_speed_right_;
  double left_wheel_pos_;
  double right_wheel_pos_;
  double x_;
  double y_;
  double theta_;
  rclcpp::Time last_time_;
  rclcpp::Time last_cmd_vel_time_;
  bool has_cmd_vel_;
  int stop_count_;

  // Encoder Tracking
  bool has_encoder_data_;
  bool has_prev_ticks_;
  int32_t prev_left_ticks_;
  int32_t prev_right_ticks_;
  double delta_s_left_;
  double delta_s_right_;

  // ROS 2 Interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_gazebo_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // MOBILE_ROBOT_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
```

#### Step 2.5-B: Create C++ Kinematics Implementation (`src/diff_drive_controller.cpp`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/src/diff_drive_controller.cpp
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/src/diff_drive_controller.cpp
```
*Paste and save:*
```cpp
/**
 * ============================================================================
 * Mobile Robot Differential Drive Controller (C++)
 * ----------------------------------------------------------------------------
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 *
 * Designed for ROS 2 Mobile Robotics Workshop.
 * This node performs:
 *  1. Inverse Kinematics: Translates /cmd_vel into Left/Right wheel angular speeds.
 *  2. Forward Kinematics (Dead-Reckoning Odometry): Computes robot pose (x, y, theta).
 *  3. Joint State Publishing: Publishes wheel positions to animate 4 wheels in RViz.
 *  4. TF Broadcasting: Broadcasts dynamic transformation 'odom' -> 'base_footprint'.
 *  5. Hardware Command Publishing: Publishes wheel speeds for Arduino firmware.
 * ============================================================================
 */

#include "mobile_robot_controller/diff_drive_controller.hpp"

using namespace std::chrono_literals;

DiffDriveControllerCpp::DiffDriveControllerCpp()
: Node("diff_drive_controller_cpp")
{
  // -------------------------------------------------------------------------
  // 1. Declare and Read ROS 2 Parameters
  // -------------------------------------------------------------------------
  this->declare_parameter<double>("wheel_radius", 0.05);         // meters
  this->declare_parameter<double>("wheel_separation", 0.34);     // meters (track width)
  this->declare_parameter<double>("publish_rate", 50.0);         // Hz
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  this->declare_parameter<int>("encoder_cpr", 330);
  this->declare_parameter<bool>("publish_joint_states", true);
  this->declare_parameter<double>("cmd_vel_timeout", 0.5);

  r_ = this->get_parameter("wheel_radius").as_double();
  L_ = this->get_parameter("wheel_separation").as_double();
  rate_ = this->get_parameter("publish_rate").as_double();
  odom_frame_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_ = this->get_parameter("base_frame_id").as_string();
  cpr_ = this->get_parameter("encoder_cpr").as_int();
  publish_joint_states_ = this->get_parameter("publish_joint_states").as_bool();
  cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();

  // -------------------------------------------------------------------------
  // 2. Initialize Variables
  // -------------------------------------------------------------------------
  cmd_linear_x_ = 0.0;
  cmd_angular_z_ = 0.0;
  wheel_speed_left_ = 0.0;
  wheel_speed_right_ = 0.0;
  left_wheel_pos_ = 0.0;
  right_wheel_pos_ = 0.0;
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  last_time_ = this->now();
  last_cmd_vel_time_ = this->now();
  has_cmd_vel_ = false;
  stop_count_ = 0;

  has_encoder_data_ = false;
  has_prev_ticks_ = false;
  prev_left_ticks_ = 0;
  prev_right_ticks_ = 0;
  delta_s_left_ = 0.0;
  delta_s_right_ = 0.0;

  // -------------------------------------------------------------------------
  // 3. Subscribers, Publishers & TF Broadcaster
  // -------------------------------------------------------------------------
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&DiffDriveControllerCpp::cmdVelCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/wheel_encoder_ticks", 10,
    std::bind(&DiffDriveControllerCpp::encoderCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed_commands", 10);
  wheel_cmd_gazebo_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_velocity_controller/commands", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Periodic Update Timer (using simulation-aware ROS clock timer for smooth TF sync)
  auto timer_period = rclcpp::Duration::from_seconds(1.0 / rate_);
  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    timer_period,
    std::bind(&DiffDriveControllerCpp::updateLoop, this));

  RCLCPP_INFO(this->get_logger(), "============================================================");
  RCLCPP_INFO(this->get_logger(), "🚀 C++ Diff Drive Controller Initialized");
  RCLCPP_INFO(this->get_logger(), "   Wheel Radius:     %.3f m", r_);
  RCLCPP_INFO(this->get_logger(), "   Wheel Separation: %.3f m", L_);
  RCLCPP_INFO(this->get_logger(), "   Update Frequency: %.1f Hz", rate_);
  RCLCPP_INFO(this->get_logger(), "============================================================");
}

void DiffDriveControllerCpp::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (std::abs(msg->linear.x) < 1e-4 && std::abs(msg->angular.z) < 1e-4) {
    if (!has_cmd_vel_) {
      return;
    }
    cmd_linear_x_ = 0.0;
    cmd_angular_z_ = 0.0;
    wheel_speed_left_ = 0.0;
    wheel_speed_right_ = 0.0;
    has_cmd_vel_ = false;
    return;
  }
  last_cmd_vel_time_ = this->now();
  has_cmd_vel_ = true;

  cmd_linear_x_ = msg->linear.x;
  cmd_angular_z_ = msg->angular.z;

  // -------------------------------------------------------------------------
  // INVERSE KINEMATICS:
  // V_left  = V - (W * L / 2)
  // V_right = V + (W * L / 2)
  // W_left  = V_left / R
  // W_right = V_right / R
  // -------------------------------------------------------------------------
  double v_left = cmd_linear_x_ - (cmd_angular_z_ * L_ / 2.0);
  double v_right = cmd_linear_x_ + (cmd_angular_z_ * L_ / 2.0);

  wheel_speed_left_ = v_left / r_;
  wheel_speed_right_ = v_right / r_;
}

void DiffDriveControllerCpp::encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    return;
  }
  int32_t left_ticks = msg->data[0];
  int32_t right_ticks = msg->data[1];

  if (has_prev_ticks_) {
    int32_t d_left = left_ticks - prev_left_ticks_;
    int32_t d_right = right_ticks - prev_right_ticks_;

    double meters_per_tick = (2.0 * M_PI * r_) / static_cast<double>(cpr_);
    delta_s_left_ = d_left * meters_per_tick;
    delta_s_right_ = d_right * meters_per_tick;
    has_encoder_data_ = true;
  }

  prev_left_ticks_ = left_ticks;
  prev_right_ticks_ = right_ticks;
  has_prev_ticks_ = true;
}

void DiffDriveControllerCpp::updateLoop()
{
  rclcpp::Time current_time = this->now();

  // -------------------------------------------------------------------------
  // Safety Watchdog: If /cmd_vel publisher stops, auto-stop robot after 0.5s
  // -------------------------------------------------------------------------
  if (has_cmd_vel_ && (current_time - last_cmd_vel_time_).seconds() > cmd_vel_timeout_) {
    cmd_linear_x_ = 0.0;
    cmd_angular_z_ = 0.0;
    wheel_speed_left_ = 0.0;
    wheel_speed_right_ = 0.0;
    has_cmd_vel_ = false;
    stop_count_ = 25;  // Actively send zero twists for 0.5s at 50Hz to halt simulation
  }

  if (stop_count_ > 0) {
    stop_count_--;
    geometry_msgs::msg::Twist stop_twist;
    cmd_vel_pub_->publish(stop_twist);

    std_msgs::msg::Float64MultiArray stop_wheel_cmds;
    stop_wheel_cmds.data = {0.0, 0.0, 0.0, 0.0};
    wheel_cmd_gazebo_pub_->publish(stop_wheel_cmds);
  }

  double dt = (current_time - last_time_).seconds();
  if (dt <= 0.0) {
    return;
  }
  if (dt > 1.0) {
    last_time_ = current_time;
    return;
  }
  last_time_ = current_time;

  // -------------------------------------------------------------------------
  // FORWARD KINEMATICS & POSE INTEGRATION:
  // -------------------------------------------------------------------------
  double delta_s = 0.0;
  double delta_theta = 0.0;
  double v_robot = 0.0;
  double w_robot = 0.0;

  if (has_encoder_data_ && (std::abs(delta_s_left_) > 1e-6 || std::abs(delta_s_right_) > 1e-6)) {
    delta_s = (delta_s_right_ + delta_s_left_) / 2.0;
    delta_theta = (delta_s_right_ - delta_s_left_) / L_;
    delta_s_left_ = 0.0;
    delta_s_right_ = 0.0;

    v_robot = delta_s / dt;
    w_robot = delta_theta / dt;
  } else {
    double v_left_linear = wheel_speed_left_ * r_;
    double v_right_linear = wheel_speed_right_ * r_;

    v_robot = (v_right_linear + v_left_linear) / 2.0;
    w_robot = (v_right_linear - v_left_linear) / L_;

    delta_s = v_robot * dt;
    delta_theta = w_robot * dt;
  }

  // Euler Integration (Midpoint Runge-Kutta 2nd Order)
  x_ += delta_s * std::cos(theta_ + delta_theta / 2.0);
  y_ += delta_s * std::sin(theta_ + delta_theta / 2.0);
  theta_ += delta_theta;

  // Normalize theta to [-pi, pi]
  theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

  // Integrate wheel rotations for RViz animation
  left_wheel_pos_ += wheel_speed_left_ * dt;
  right_wheel_pos_ += wheel_speed_right_ * dt;

  // Quaternion from yaw
  double qz = std::sin(theta_ / 2.0);
  double qw = std::cos(theta_ / 2.0);

  // -------------------------------------------------------------------------
  // 1. Publish TF: 'odom' -> 'base_footprint'
  // -------------------------------------------------------------------------
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = current_time;
  t.header.frame_id = odom_frame_;
  t.child_frame_id = base_frame_;

  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;

  tf_broadcaster_->sendTransform(t);

  // -------------------------------------------------------------------------
  // 2. Publish /odom Topic
  // -------------------------------------------------------------------------
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;

  // Set covariance to prevent oversized RViz covariance bubbles
  odom.pose.covariance = {
    0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
    0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
    0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
    0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
    0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
    0.0,   0.0,   0.0, 0.0, 0.0, 0.01
  };

  odom.twist.twist.linear.x = v_robot;
  odom.twist.twist.angular.z = w_robot;
  odom.twist.covariance = odom.pose.covariance;

  odom_pub_->publish(odom);

  // -------------------------------------------------------------------------
  // 3. Publish /joint_states Topic (all 4 wheels)
  // -------------------------------------------------------------------------
  sensor_msgs::msg::JointState js;
  js.header.stamp = current_time;
  js.name = {
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint"
  };
  js.position = {left_wheel_pos_, right_wheel_pos_, left_wheel_pos_, right_wheel_pos_};
  js.velocity = {wheel_speed_left_, wheel_speed_right_, wheel_speed_left_, wheel_speed_right_};

  if (publish_joint_states_) {
    joint_state_pub_->publish(js);
  }

  // -------------------------------------------------------------------------
  // 4. Publish /wheel_speed_commands (for Arduino firmware)
  // -------------------------------------------------------------------------
  std_msgs::msg::Float32MultiArray wheel_cmds;
  wheel_cmds.data = {static_cast<float>(wheel_speed_left_), static_cast<float>(wheel_speed_right_)};
  wheel_cmd_pub_->publish(wheel_cmds);

  // 5. Publish to Gazebo actuator controller (/joint_group_velocity_controller/commands)
  std_msgs::msg::Float64MultiArray gazebo_wheel_cmds;
  gazebo_wheel_cmds.data = {wheel_speed_left_, wheel_speed_right_, wheel_speed_left_, wheel_speed_right_};
  wheel_cmd_gazebo_pub_->publish(gazebo_wheel_cmds);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiffDriveControllerCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

#### Step 2.6-B: Create Python Kinematics Implementation (`scripts/diff_drive_controller.py`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/scripts/diff_drive_controller.py
chmod +x ~/ros2_mobile_robot_ws/src/mobile_robot_controller/scripts/diff_drive_controller.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/scripts/diff_drive_controller.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Mobile Robot Differential Drive Controller (Python)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Designed for ROS 2 Mobile Robotics Workshop.
This node performs:
 1. Inverse Kinematics: Translates /cmd_vel (Twist) into Left/Right wheel angular speeds.
 2. Forward Kinematics (Dead-Reckoning Odometry): Computes robot pose (x, y, theta).
 3. Joint State Publishing: Publishes wheel rotations to animate all 4 wheels in RViz.
 4. TF Broadcasting: Broadcasts the dynamic transformation 'odom' -> 'base_footprint'.
 5. Hardware Command Publishing: Publishes wheel speeds for the Arduino firmware.
================================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from tf2_ros import TransformBroadcaster


class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # ---------------------------------------------------------------------
        # 1. Declare and Read ROS 2 Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter('wheel_radius', 0.05)         # meters
        self.declare_parameter('wheel_separation', 0.34)     # meters (track width)
        self.declare_parameter('publish_rate', 30.0)         # Hz
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('encoder_cpr', 330)           # Counts per rev
        self.declare_parameter('publish_joint_states', True)

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.cpr = self.get_parameter('encoder_cpr').get_parameter_value().integer_value
        self.publish_joint_states = self.get_parameter('publish_joint_states').get_parameter_value().bool_value
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value

        # Encoder tracking
        self.has_encoder_data = False
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.delta_s_left = 0.0
        self.delta_s_right = 0.0

        # ---------------------------------------------------------------------
        # 2. Internal Kinematic State Variables
        # ---------------------------------------------------------------------
        # Target velocities from /cmd_vel
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # Wheel angular velocities (rad/s)
        self.wheel_speed_left = 0.0
        self.wheel_speed_right = 0.0

        # Accumulated wheel angular positions (radians, for joint_states)
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Integrated Robot Pose in Odom frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timestamp tracking for numerical integration (dt)
        self.last_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()
        self.has_cmd_vel = False

        # ---------------------------------------------------------------------
        # 3. Subscribers and Publishers
        # ---------------------------------------------------------------------
        from std_msgs.msg import Int32MultiArray

        # Subscribe to velocity commands (from keyboard teleop or navigation)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher to /cmd_vel for safety watchdog auto-stop in Gazebo
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Gazebo actuator velocity command publisher
        self.wheel_cmd_gazebo_pub = self.create_publisher(Float64MultiArray, '/joint_group_velocity_controller/commands', 10)

        # Subscribe to wheel encoder ticks for closed-loop odometry
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/wheel_encoder_ticks',
            self.encoder_ticks_callback,
            10
        )

        # Publish Odometry for navigation algorithms (SLAM / Nav2)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Publish Joint States so RViz rotates the 3D wheel meshes
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Publish Wheel Speed Commands for microcontroller / Arduino firmware
        self.wheel_cmd_pub = self.create_publisher(Float32MultiArray, '/wheel_speed_commands', 10)

        # TF Broadcaster for 'odom' -> 'base_footprint'
        self.tf_broadcaster = TransformBroadcaster(self)

        # Periodic Timer Loop (using simulation-aware ROS clock)
        self.timer = self.create_timer(1.0 / self.rate, self.update_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 Python Diff Drive Controller Initialized')
        self.get_logger().info(f'   Wheel Radius:     {self.r} m')
        self.get_logger().info(f'   Wheel Separation: {self.L} m')
        self.get_logger().info(f'   Update Frequency: {self.rate} Hz')
        self.get_logger().info('=' * 60)

    def cmd_vel_callback(self, msg: Twist):
        """Callback executed whenever a new /cmd_vel Twist message arrives."""
        if abs(msg.linear.x) < 1e-4 and abs(msg.angular.z) < 1e-4 and not self.has_cmd_vel:
            return
        self.last_cmd_vel_time = self.get_clock().now()
        self.has_cmd_vel = True

        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

        # ---------------------------------------------------------------------
        # INVERSE KINEMATICS:
        # Given desired robot linear velocity V (m/s) and angular velocity W (rad/s):
        #   V_left  = V - (W * L / 2)
        #   V_right = V + (W * L / 2)
        # Rotational speeds (rad/s):
        #   W_left  = V_left / R
        #   W_right = V_right / R
        # ---------------------------------------------------------------------
        v_left = self.cmd_linear_x - (self.cmd_angular_z * self.L / 2.0)
        v_right = self.cmd_linear_x + (self.cmd_angular_z * self.L / 2.0)

        self.wheel_speed_left = v_left / self.r
        self.wheel_speed_right = v_right / self.r

    def encoder_ticks_callback(self, msg):
        """Processes real hardware encoder tick counts."""
        if len(msg.data) < 2:
            return

        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        if self.prev_left_ticks is not None and self.prev_right_ticks is not None:
            d_left = left_ticks - self.prev_left_ticks
            d_right = right_ticks - self.prev_right_ticks

            # Distance moved by each wheel side (meters)
            meters_per_tick = (2.0 * math.pi * self.r) / float(self.cpr)
            self.delta_s_left = d_left * meters_per_tick
            self.delta_s_right = d_right * meters_per_tick
            self.has_encoder_data = True

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

    def update_loop(self):
        """Periodic loop: updates odometry, publishes wheel commands and TF."""
        current_time = self.get_clock().now()

        # ---------------------------------------------------------------------
        # Safety Watchdog: If /cmd_vel publisher stops, auto-stop robot after 0.5s
        # ---------------------------------------------------------------------
        if self.has_cmd_vel and (current_time - self.last_cmd_vel_time).nanoseconds / 1e9 > self.cmd_vel_timeout:
            if abs(self.cmd_linear_x) > 1e-4 or abs(self.cmd_angular_z) > 1e-4:
                self.cmd_linear_x = 0.0
                self.cmd_angular_z = 0.0
                self.wheel_speed_left = 0.0
                self.wheel_speed_right = 0.0
                self.has_cmd_vel = False

                # Halt Gazebo physics simulation immediately
                stop_twist = Twist()
                self.cmd_vel_pub.publish(stop_twist)

                stop_msg = Float64MultiArray()
                stop_msg.data = [0.0, 0.0, 0.0, 0.0]
                self.wheel_cmd_gazebo_pub.publish(stop_msg)

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        if dt > 1.0:
            self.last_time = current_time
            return
        self.last_time = current_time

        # ---------------------------------------------------------------------
        # FORWARD KINEMATICS & POSE INTEGRATION:
        # If real encoder data is available, compute displacement from encoders.
        # Otherwise, fall back to velocity command dead-reckoning.
        # ---------------------------------------------------------------------
        if self.has_encoder_data and (abs(self.delta_s_left) > 1e-6 or abs(self.delta_s_right) > 1e-6):
            delta_s = (self.delta_s_right + self.delta_s_left) / 2.0
            delta_theta = (self.delta_s_right - self.delta_s_left) / self.L
            # Reset delta for next tick cycle
            self.delta_s_left = 0.0
            self.delta_s_right = 0.0

            v_robot = delta_s / dt
            w_robot = delta_theta / dt
        else:
            v_left_linear = self.wheel_speed_left * self.r
            v_right_linear = self.wheel_speed_right * self.r
            v_robot = (v_right_linear + v_left_linear) / 2.0
            w_robot = (v_right_linear - v_left_linear) / self.L

            delta_s = v_robot * dt
            delta_theta = w_robot * dt

        # Midpoint / Euler numerical integration
        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Integrate wheel angular positions for RViz wheel spinning animation
        self.left_wheel_pos += self.wheel_speed_left * dt
        self.right_wheel_pos += self.wheel_speed_right * dt

        # Convert Euler yaw (theta) to Quaternion: (qx, qy, qz, qw)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # ---------------------------------------------------------------------
        # 1. Publish TF: 'odom' -> 'base_footprint'
        # ---------------------------------------------------------------------
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # ---------------------------------------------------------------------
        # 2. Publish /odom Topic
        # ---------------------------------------------------------------------
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose in odom frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Accurate small covariance to prevent huge RViz covariance bubbles
        odom_msg.pose.covariance = [
            0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
            0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
            0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
            0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
            0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
            0.0,   0.0,   0.0, 0.0, 0.0, 0.01
        ]

        # Velocity in base_footprint frame
        odom_msg.twist.twist.linear.x = v_robot
        odom_msg.twist.twist.angular.z = w_robot
        odom_msg.twist.covariance = odom_msg.pose.covariance

        self.odom_pub.publish(odom_msg)

        # ---------------------------------------------------------------------
        # 3. Publish /joint_states Topic (all 4 wheels)
        # ---------------------------------------------------------------------
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        joint_state.position = [
            self.left_wheel_pos,
            self.right_wheel_pos,
            self.left_wheel_pos,
            self.right_wheel_pos
        ]
        joint_state.velocity = [
            self.wheel_speed_left,
            self.wheel_speed_right,
            self.wheel_speed_left,
            self.wheel_speed_right
        ]
        if self.publish_joint_states:
            self.joint_state_pub.publish(joint_state)

        # ---------------------------------------------------------------------
        # 4. Publish /wheel_speed_commands (for Arduino firmware)
        # ---------------------------------------------------------------------
        wheel_cmds = Float32MultiArray()
        # [left_rad_s, right_rad_s]
        wheel_cmds.data = [float(self.wheel_speed_left), float(self.wheel_speed_right)]
        self.wheel_cmd_pub.publish(wheel_cmds)

        # 5. Publish to Gazebo actuator controller (/joint_group_velocity_controller/commands)
        gazebo_cmds = Float64MultiArray()
        gazebo_cmds.data = [
            float(self.wheel_speed_left),
            float(self.wheel_speed_right),
            float(self.wheel_speed_left),
            float(self.wheel_speed_right)
        ]
        self.wheel_cmd_gazebo_pub.publish(gazebo_cmds)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
```

#### Step 2.7-B: Create Kinematics Launch File (`launch/controller.launch.py`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/controller.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/controller.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_controller')
    default_params = os.path.join(pkg_share, 'config', 'controller_params.yaml')

    # Launch Arguments
    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Choose C++ controller (true) or Python controller (false)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true (default: true)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to controller_params.yaml'
    )

    # 1. C++ Controller Node
    cpp_controller_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller_cpp',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('use_cpp'))
    )

    # 2. Python Controller Node
    py_controller_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller_py',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=UnlessCondition(LaunchConfiguration('use_cpp'))
    )

    return LaunchDescription([
        use_cpp_arg,
        use_sim_time_arg,
        params_file_arg,
        cpp_controller_node,
        py_controller_node
    ])
```

#### Step 2.8-B: Configure Option 2 `package.xml` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/package.xml
```
*Replace contents with Option 2 pure coding package manifest:*
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mobile_robot_controller</name>
  <version>1.0.0</version>
  <description>Kinematics and odometry controller in C++ and Python for 4-wheel robot.</description>
  <maintainer email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</maintainer>
  <author email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</author>
  <license>Apache-2.0</license>

  <url type="website">https://rahul-r-joshua.github.io/portfolio/</url>
  <url type="repository">https://github.com/rahul-r-joshua</url>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Core Kinematics Dependencies -->
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### Step 2.9-B: Configure Option 2 `CMakeLists.txt` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/CMakeLists.txt
```
*Replace contents with Option 2 build instructions:*
```cmake
cmake_minimum_required(VERSION 3.8)
project(mobile_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 1. C++ Controller Executable
add_executable(diff_drive_controller_cpp src/diff_drive_controller.cpp)
target_include_directories(diff_drive_controller_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diff_drive_controller_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# Install C++ binary
install(TARGETS
  diff_drive_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include
)

# 2. Python Scripts (Install both original name and _py alias for launch file compatibility)
install(PROGRAMS
  scripts/diff_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME diff_drive_controller_py
)

install(PROGRAMS
  scripts/diff_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install Config and Launch directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

#### Step 2.10-B: Build Option 2
```bash
cd ~/ros2_mobile_robot_ws
colcon build --packages-select mobile_robot_controller --symlink-install
source install/setup.bash
```

---

#### ⚡ Option 2 Verification (Terminal-by-Terminal Execution)

> [!IMPORTANT]
> **⚠️ Beginner Rule: In EVERY new terminal tab you open, always run:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_mobile_robot_ws/install/setup.bash
> ```

##### Terminal 1: Launch Gazebo Simulation (1st: Gazebo)
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
source install/setup.bash
ros2 launch mobile_robot_description gazebo.launch.py
```

##### Terminal 2: Launch Custom Kinematics Controller (2nd: Controller)
* **To run the C++ Kinematics Node:**
  ```bash
  cd ~/ros2_mobile_robot_ws
  source install/setup.bash
  ros2 launch mobile_robot_controller controller.launch.py use_cpp:=true
  ```
* **To run the Python Kinematics Node:**
  ```bash
  cd ~/ros2_mobile_robot_ws
  source install/setup.bash
  ros2 launch mobile_robot_controller controller.launch.py use_cpp:=false
  ```

##### Terminal 3: Launch RViz Visualization (3rd: Display / RViz)
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_description display.launch.py
```

##### Terminal 4: Test Kinematic Motions (`ros2 topic pub`)
*(Tip: Using `--times 20` sends 20 messages at 10 Hz for 2 seconds and returns to the shell prompt automatically).*
* **1. Forward Drive (+0.3 m/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
  ```
* **2. Backward Drive (-0.3 m/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.2}, angular: {z: 0.0}}"
  ```
* **3. Left Turn In-Place (+0.5 rad/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.4}}"
  ```
* **4. Right Turn In-Place (-0.5 rad/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.4}}"
  ```
* **5. Forward Arc with Right Turn (+0.3 m/s, -0.3 rad/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: -0.3}}"
  ```
* **6. Backward Arc with Left Turn (-0.3 m/s, +0.3 rad/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.2}, angular: {z: 0.3}}"
  ```
*(Note: If running without `--times 20`, press `Ctrl+C` after any command to verify the 0.5-second watchdog stop automatically halts the robot).*

##### Terminal 5: Echo Calculated Odometry (`/odom`)
```bash
source /opt/ros/humble/setup.bash
# Single-shot odometry inspection:
ros2 topic echo /odom --once
```

---

### ⚙️ OPTION 3: Pure ROS 2 Controller Manager Track (`ros2_control` Framework Only)

*Follow this track starting immediately after Milestone 1 if you want to skip custom kinematics programming and use only official, upstream ROS 2 Control packages with `controller_manager`.*

#### Step 2.1-C: Create Package with `ros2 pkg create`
```bash
cd ~/ros2_mobile_robot_ws/src
ros2 pkg create --build-type ament_cmake mobile_robot_controller
```

#### Step 2.2-C: Create Subdirectories
```bash
cd ~/ros2_mobile_robot_ws/src/mobile_robot_controller
mkdir -p config launch
```

#### Step 2.3-C: Create `ros2_control` Parameters (`config/ros2_robot_controller_params.yaml`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/ros2_robot_controller_params.yaml
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/ros2_robot_controller_params.yaml
```
*Paste and save:*
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_group_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

joint_group_velocity_controller:
  ros__parameters:
    joints:
      - front_left_wheel_joint
      - front_right_wheel_joint
      - rear_left_wheel_joint
      - rear_right_wheel_joint

diff_drive_controller:
  ros__parameters:
    use_sim_time: true
    left_wheel_names:
      - front_left_wheel_joint
      - rear_left_wheel_joint
    right_wheel_names:
      - front_right_wheel_joint
      - rear_right_wheel_joint

    wheel_separation: 0.34
    wheel_radius: 0.05

    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0

    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_footprint
    pose_covariance_diagonal: [0.001, 0.001, 1.0e-3, 1.0e-3, 1.0e-3, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 1.0e-3, 1.0e-3, 1.0e-3, 0.01]

    open_loop: false
    enable_odom_tf: true

    cmd_vel_timeout: 0.5
    use_stamped_vel: false

    # Velocity and acceleration limits
    linear:
      x:
        has_velocity_limits: true
        max_velocity: 0.3
        min_velocity: -0.3
        has_acceleration_limits: true
        max_acceleration: 1.0
        min_acceleration: -1.0
    angular:
      z:
        has_velocity_limits: true
        max_velocity: 0.6
        min_velocity: -0.6
        has_acceleration_limits: true
        max_acceleration: 2.0
        min_acceleration: -2.0
```

#### Step 2.4-C: Create Controller Spawner Launch File (`launch/ros2_controller.launch.py`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/ros2_controller.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/ros2_controller.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
ros2_control Controller Spawner Launch
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Designed for ROS 2 Mobile Robotics Workshop.
Spawns:
  1. joint_state_broadcaster
  2. diff_drive_controller
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--unload-on-kill',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--unload-on-kill',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])
```

#### Step 2.5-C: Configure Option 3 `package.xml` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/package.xml
```
*Replace contents with Option 3 package manifest:*
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mobile_robot_controller</name>
  <version>1.0.0</version>
  <description>ROS 2 Control integration for 4-wheel mobile robot using controller_manager.</description>
  <maintainer email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</maintainer>
  <author email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</author>
  <license>Apache-2.0</license>

  <url type="website">https://rahul-r-joshua.github.io/portfolio/</url>
  <url type="repository">https://github.com/rahul-r-joshua</url>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Option 3: Production ros2_control Dependencies -->
  <depend>controller_manager</depend>
  <depend>diff_drive_controller</depend>
  <depend>joint_state_broadcaster</depend>
  <depend>joint_state_publisher</depend>
  <depend>robot_state_publisher</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### Step 2.6-C: Configure Option 3 `CMakeLists.txt` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/CMakeLists.txt
```
*Replace contents with Option 3 build instructions:*
```cmake
cmake_minimum_required(VERSION 3.8)
project(mobile_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Config and Launch directories for ros2_control
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

#### Step 2.7-C: Build Option 3
```bash
cd ~/ros2_mobile_robot_ws
colcon build --packages-select mobile_robot_controller --symlink-install
source install/setup.bash
```

---

#### ⚡ Option 3 Verification (Terminal-by-Terminal Execution)

> [!IMPORTANT]
> **⚠️ Beginner Rule: In EVERY new terminal tab you open, always run:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_mobile_robot_ws/install/setup.bash
> ```

##### Terminal 1: Launch Gazebo Physics Simulation (1st: Gazebo)
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
source install/setup.bash
ros2 launch mobile_robot_description gazebo.launch.py
```

##### Terminal 2: Spawn `ros2_control` Controllers (2nd: Controller)
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_controller ros2_controller.launch.py
```
*(Spawns `joint_state_broadcaster` and `diff_drive_controller` via `controller_manager`.)*

##### Terminal 3: Launch RViz Visualization (3rd: Display / RViz)
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_description display.launch.py
```

##### Terminal 4: Test Driving Commands (`ros2 topic pub`)
*(Tip: Using `--times 20` sends 20 messages at 10 Hz for 2.0 seconds and returns to the shell prompt automatically).*
* **1. Forward Drive (+0.4 m/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.0}}"
  ```
* **2. Backward Drive (-0.4 m/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.4}, angular: {z: 0.0}}"
  ```
* **3. Left Turn In-Place (+0.8 rad/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.8}}"
  ```
* **4. Right Turn In-Place (-0.8 rad/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.8}}"
  ```
* **5. Forward Arc with Right Turn (+0.3 m/s, -0.4 rad/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: -0.4}}"
  ```
* **6. Backward Arc with Left Turn (-0.3 m/s, +0.4 rad/s for 2s):**
  ```bash
  ros2 topic pub --times 20 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}, angular: {z: 0.4}}"
  ```

##### Terminal 5: Echo Odometry (`/odom`)
```bash
source /opt/ros/humble/setup.bash
# Single-shot odometry inspection:
ros2 topic echo /odom --once
```

##### Terminal 6: Inspect Active Controllers & Hardware Interfaces
```bash
source /opt/ros/humble/setup.bash
ros2 control list_controllers
```
*Expected output:*
```text
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
diff_drive_controller[diff_drive_controller/DiffDriveController] active
```

```bash
ros2 control list_hardware_interfaces
```
*Expected output: Wheel joints `front_left`, `front_right`, `rear_left`, `rear_right` with `velocity` command interface and `position, velocity` state interfaces.*

---

<a id="milestone-3" name="milestone-3"></a>
## Milestone 3
### Teleoperation, Joystick & Twist Mux (`mobile_robot_controller`)

In Milestone 3, you add gamepad teleoperation with Deadman failsafe switches, priority velocity multiplexing (`twist_mux`), and topic translation relays.

### Step 3.1: Create `config/joy_config.yaml`
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/joy_config.yaml
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/joy_config.yaml
```
*Paste and save:*
```yaml
/**:
  ros__parameters:
    device_id: 0
    device_name: ""
    deadzone: 0.05
    autorepeat_rate: 30.0
    sticky_buttons: false
    coalesce_interval_ms: 1
```

### Step 3.2: Create `config/joy_teleop.yaml` (Left Stick Drive, Right Stick Turn, LB Deadman)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/joy_teleop.yaml
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/joy_teleop.yaml
```
*Paste and save:*
```yaml
/**:
  ros__parameters:
    axis_linear:
      x: 1                # Left stick vertical (forward / backward)
    scale_linear:
      x: 0.3              # Reduced normal speed: 0.3 m/s (safe & smooth indoors)
    scale_linear_turbo:
      x: 0.6              # Reduced turbo speed: 0.6 m/s

    axis_angular:
      yaw: 3              # Right stick horizontal (turning / steering)
    scale_angular:
      yaw: 0.4            # Very smooth normal turning rate: 0.4 rad/s
    scale_angular_turbo:
      yaw: 0.8            # Controlled turbo turning rate: 0.8 rad/s

    enable_button: 4        # Deadman switch: LB (Button 4) - Must hold to drive
    enable_turbo_button: 5  # Turbo switch: RB (Button 5) - Hold for turbo speed
    require_enable_button: true
```

### Step 3.3: Create `config/twist_mux_topics.yaml`
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/twist_mux_topics.yaml
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/twist_mux_topics.yaml
```
*Paste and save:*
```yaml
twist_mux:
  ros__parameters:
    use_stamped: false
    topics:
      joystick:
        topic   : joy_vel
        timeout : 1.5
        priority: 99
      keyboard: 
        topic   : key_vel
        timeout : 0.5
        priority: 90
      navigation: 
        topic   : cmd_vel_nav
        timeout : 0.5
        priority: 10
```

### Step 3.4: Create `config/twist_mux_locks.yaml`
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/twist_mux_locks.yaml
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/twist_mux_locks.yaml
```
*Paste and save:*
```yaml
twist_mux:
  ros__parameters:
    locks:
      safety_stop:
        topic   : safety_stop
        timeout : 0.0
        priority: 255
```

### Step 3.5: Create `config/twist_mux_joy.yaml`
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/twist_mux_joy.yaml
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/config/twist_mux_joy.yaml
```
*Paste and save:*
```yaml
joystick_relay:
  ros__parameters:
    priority: True
    turbo:
      linear_forward_min  : 0.25
      linear_forward_max  : 0.6
      linear_backward_min : 0.25
      linear_backward_max : 0.6
      angular_min : 0.5
      angular_max : 1.5
      steps       : 3
```

### Step 3.6: Create `scripts/twist_relay.py`
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/scripts/twist_relay.py
chmod +x ~/ros2_mobile_robot_ws/src/mobile_robot_controller/scripts/twist_relay.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/scripts/twist_relay.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Twist Relay Node
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Relays velocity commands between stamped and unstamped formats:
 1. /input_joy/cmd_vel_stamped (TwistStamped from joy_teleop) -> /joy_vel (Twist for twist_mux)
 2. /input_joy/cmd_vel -> /joy_vel
 3. /cmd_vel_raw (Twist) -> /cmd_vel_stamped (TwistStamped)
================================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay")

        # Subscribes to joy_teleop output (stamped) and publishes unstamped for twist_mux
        self.joy_sub = self.create_subscription(
            TwistStamped,
            "/input_joy/cmd_vel_stamped",
            self.joy_twist_callback,
            10
        )
        self.joy_pub = self.create_publisher(
            Twist,
            "/joy_vel",
            10
        )

        # Also support /input_joy/cmd_vel (unstamped input)
        self.joy_unstamped_sub = self.create_subscription(
            Twist,
            "/input_joy/cmd_vel",
            self.joy_unstamped_callback,
            10
        )

        # Controller / cmd_vel relay
        self.controller_sub = self.create_subscription(
            Twist,
            "/cmd_vel_raw",
            self.controller_twist_callback,
            10
        )
        self.controller_pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel_stamped",
            10
        )

        self.get_logger().info("Twist Relay Node Initialized.")

    def joy_twist_callback(self, msg: TwistStamped):
        twist = Twist()
        twist = msg.twist
        self.joy_pub.publish(twist)

    def joy_unstamped_callback(self, msg: Twist):
        self.joy_pub.publish(msg)

    def controller_twist_callback(self, msg: Twist):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.controller_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Step 3.7: Update `CMakeLists.txt` to Install `twist_relay.py`
Open `CMakeLists.txt`:
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/CMakeLists.txt
```
Ensure your complete `CMakeLists.txt` includes both Python scripts and C++ targets:
```cmake
cmake_minimum_required(VERSION 3.8)
project(mobile_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 1. C++ Controller Executable
add_executable(diff_drive_controller_cpp src/diff_drive_controller.cpp)
target_include_directories(diff_drive_controller_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diff_drive_controller_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# Install C++ binary
install(TARGETS
  diff_drive_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include
)

# 2. Python Scripts Installation
install(PROGRAMS
  scripts/diff_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME diff_drive_controller_py
)

install(PROGRAMS
  scripts/diff_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install Config, Launch directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### Step 3.8: Create `launch/joystick_teleop.launch.py`
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/joystick_teleop.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_controller/launch/joystick_teleop.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Joystick Teleop Launch File (matching rocker_controller architecture)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_pkg = get_package_share_directory('mobile_robot_controller')
    twist_mux_pkg = get_package_share_directory('twist_mux')

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time"
    )

    cmd_vel_out_arg = DeclareLaunchArgument(
        name="cmd_vel_out",
        default_value="/cmd_vel",
        description="Output topic for twist_mux (default: /cmd_vel; /cmd_vel_raw when safety controller is active)"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(controller_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    # teleop node (reads /joy, checks deadman button 4 LB, outputs /joy_vel)
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            os.path.join(controller_pkg, "config", "joy_teleop.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        remappings=[('/cmd_vel', '/joy_vel')]
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings=[("/cmd_vel_out", LaunchConfiguration("cmd_vel_out"))],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(controller_pkg, "config", "twist_mux_locks.yaml"),
            os.path.join(controller_pkg, "config", "twist_mux_topics.yaml"),
        ]
    )

    twist_marker_node = Node(
        package="twist_mux",
        executable="twist_marker",
        output="screen",
        remappings=[("/twist", LaunchConfiguration("cmd_vel_out"))],
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "frame_id": "base_link",
            "scale": 1.0,
            "vertical_position": 2.0
        }]
    )

    twist_relay_node = Node(
        package="mobile_robot_controller",
        executable="twist_relay.py",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        cmd_vel_out_arg,
        joy_node,
        teleop_node,
        twist_mux_node,
        twist_marker_node,
        twist_relay_node,
    ])
```

### Step 3.9: Rebuild `mobile_robot_controller`
```bash
cd ~/ros2_mobile_robot_ws
colcon build --packages-select mobile_robot_controller --symlink-install
source install/setup.bash
```

---

### ⚡ MILESTONE 3 VERIFICATION (Terminal-by-Terminal Execution)

> [!IMPORTANT]
> **⚠️ Beginner Rule: In EVERY new terminal tab you open, always run:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_mobile_robot_ws/install/setup.bash
> ```
> 
> **Strict Execution Sequence:**  
> **1st: Gazebo** ➡️ **2nd: Controller** ➡️ **3rd: Display / RViz** ➡️ **4th: Joy (Joystick Teleop)**  
> 
> *Follow this exact terminal order every time:*  
> 1. **Gazebo** initializes simulation physics and publishes `/clock`.  
> 2. **Controller** activates `joint_state_broadcaster` and `diff_drive_controller`, broadcasting the `odom -> base_footprint` transform.  
> 3. **Display / RViz** connects to simulation clock and visualizes the model and transforms.  
> 4. **Joy** launches gamepad drivers, Deadman switch, and velocity multiplexing.

#### Terminal 1: Launch Simulation Engine (1st: Gazebo / Ignition / Gz)
* **Option A: Gazebo Classic (Gazebo 11):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
source install/setup.bash
ros2 launch mobile_robot_description gazebo.launch.py
```

* **Option B: Ignition Gazebo (Fortress):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_description ign_gazebo.launch.py
```

* **Option C: Modern Gazebo (Gz Sim / Harmonic / Garden):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_description ign_gazebo.launch.py backend:=gz
```

#### Terminal 2: Launch Controller (2nd: Controller)
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_controller ros2_controller.launch.py
```
*(Once started, `joint_state_broadcaster` activates `/joint_states` and all wheel transforms resolve instantly in RViz).*

#### Terminal 3: Launch RViz Visualization (3rd: Display / RViz)
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_description display.launch.py
```

#### Terminal 4: Launch Joystick Teleop & Twist Mux (4th: Joy)
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_controller joystick_teleop.launch.py
```

#### Terminal 5: Test Driving (Choose Gamepad or Terminal Mode)

##### Option A: Using Physical Gamepad / Joystick Controller
* **Normal Mode (0.3 m/s):** Hold **LB (Button 4: Deadman Switch)** and push the **Left Analog Stick** forward to drive, and **Right Analog Stick** left/right to steer.
* **Turbo Mode (0.6 m/s - 2x Speed):** Hold **RB (Button 5: Turbo Switch)** while deflecting the stick for high-speed indoor driving.
* **Deadman Failsafe:** Release the button at any time to instantly stop the robot.

##### Option B: Using Terminal Commands (No Gamepad Required)
If you do not have a physical gamepad plugged in, test both modes directly from the terminal (press `Ctrl+C` to stop driving):

* **1. Test Normal Mode (0.3 m/s - Simulates holding LB Button 4):**
  ```bash
  ros2 topic pub -r 10 /joy sensor_msgs/msg/Joy '{axes: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 1, 0, 0, 0]}'
  ```

* **2. Test Turbo Mode (0.6 m/s - Simulates holding RB Button 5):**
  ```bash
  ros2 topic pub -r 10 /joy sensor_msgs/msg/Joy '{axes: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 1, 0, 0]}'
  ```

* **3. Test Keyboard Teleop via Twist Mux (Priority 90):**
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
  ```

* **4. Echo Resulting Driving Velocity (`/cmd_vel`):**
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic echo /cmd_vel
  ```

> [!NOTE]
> **Deadman Safety Protocol:**  
> `teleop_twist_joy` enforces a hardware Deadman switch (`enable_button: 4` / LB). Deflecting the joystick without depressing LB will not produce driving commands. This prevents accidental robot motion when picking up or bumping the gamepad.

---

<a id="milestone-4" name="milestone-4"></a>
## Milestone 4
### Active Safety Zones & LiDAR Collision Avoidance (`mobile_robot_bringup`)

In Milestone 4, you build an active safety zone obstacle interception node. If obstacles enter the warning zone ($0.9\text{m}$), linear velocity is automatically reduced by 50%. If obstacles enter the emergency stop zone ($0.45\text{m}$), commands are locked to zero and an active emergency stop signal (`/safety_stop`) is asserted to lock `twist_mux`.

We provide **both modern C++ (`rclcpp`) and Python (`rclpy`)** implementations.

### Step 4.1: Create the Package with `ros2 pkg create`
```bash
cd ~/ros2_mobile_robot_ws/src
ros2 pkg create --build-type ament_cmake mobile_robot_bringup
```

### Step 4.2: Create Subdirectories
```bash
cd ~/ros2_mobile_robot_ws/src/mobile_robot_bringup
mkdir -p include/mobile_robot_bringup src scripts launch config
```

### Step 4.3: Create C++ Safety Zone Header (`include/mobile_robot_bringup/safety_zone_controller.hpp`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/include/mobile_robot_bringup/safety_zone_controller.hpp
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/include/mobile_robot_bringup/safety_zone_controller.hpp
```
*Paste and save:*
```cpp
#ifndef MOBILE_ROBOT_BRINGUP__SAFETY_ZONE_CONTROLLER_HPP_
#define MOBILE_ROBOT_BRINGUP__SAFETY_ZONE_CONTROLLER_HPP_

#include <cmath>
#include <string>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mobile_robot_bringup
{

class SafetyZoneControllerCpp : public rclcpp::Node
{
public:
  SafetyZoneControllerCpp();
  virtual ~SafetyZoneControllerCpp() = default;

private:
  // ROS Callbacks
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cmdRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void safetyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void controlLoop();

  // Helper Methods
  void publishMarkers();

  // Subscribers & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_raw_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_joy_vel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_safe_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double red_dist_;
  double yellow_dist_;
  double fov_rad_;
  double slowdown_factor_;
  double lidar_x_offset_;

  // State
  double min_obstacle_distance_;
  std::string zone_state_;
  geometry_msgs::msg::Twist raw_cmd_;
  rclcpp::Time last_cmd_time_;
  bool manual_safety_active_{false};
  bool has_fresh_raw_cmd_{false};
};

}  // namespace mobile_robot_bringup

#endif  // MOBILE_ROBOT_BRINGUP__SAFETY_ZONE_CONTROLLER_HPP_
```

### Step 4.4: Create C++ Safety Zone Implementation (`src/safety_zone_controller.cpp`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/src/safety_zone_controller.cpp
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/src/safety_zone_controller.cpp
```
*Paste and save:*
```cpp
#include "mobile_robot_bringup/safety_zone_controller.hpp"

namespace mobile_robot_bringup
{

SafetyZoneControllerCpp::SafetyZoneControllerCpp()
: Node("safety_zone_controller"),
  min_obstacle_distance_(std::numeric_limits<double>::infinity()),
  zone_state_("GREEN")
{
  // 1. Declare & Get Parameters
  this->declare_parameter<double>("red_zone_distance", 0.45);
  this->declare_parameter<double>("yellow_zone_distance", 0.90);
  this->declare_parameter<double>("fov_angle_deg", 90.0);
  this->declare_parameter<double>("slowdown_factor", 0.5);
  this->declare_parameter<double>("lidar_x_offset", 0.10);

  red_dist_ = this->get_parameter("red_zone_distance").as_double();
  yellow_dist_ = this->get_parameter("yellow_zone_distance").as_double();
  double fov_deg = this->get_parameter("fov_angle_deg").as_double();
  fov_rad_ = fov_deg * M_PI / 180.0;
  slowdown_factor_ = this->get_parameter("slowdown_factor").as_double();
  lidar_x_offset_ = this->get_parameter("lidar_x_offset").as_double();

  last_cmd_time_ = this->now();

  // 2. Subscribers
  sub_cmd_raw_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_raw", 10,
    std::bind(&SafetyZoneControllerCpp::cmdRawCallback, this, std::placeholders::_1));

  sub_joy_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/joy_vel", 10,
    std::bind(&SafetyZoneControllerCpp::cmdRawCallback, this, std::placeholders::_1));

  // Use SensorDataQoS (Best Effort) for universal compatibility with Gazebo & physical LiDARs (RPLiDAR/YDLidar)
  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&SafetyZoneControllerCpp::scanCallback, this, std::placeholders::_1));

  sub_safety_stop_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_stop", 10,
    std::bind(&SafetyZoneControllerCpp::safetyStopCallback, this, std::placeholders::_1));

  // 3. Publishers
  pub_cmd_safe_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/safety_zone_markers", 10);

  // 4. Periodic Timer at 30 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(33),
    std::bind(&SafetyZoneControllerCpp::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "============================================================");
  RCLCPP_INFO(this->get_logger(), "🛡️  Safety Zone Controller (C++) Initialized");
  RCLCPP_INFO(this->get_logger(), "   🔴 Red Zone (1x Robot Size):    %.2f m  -> FULL STOP", red_dist_);
  RCLCPP_INFO(this->get_logger(), "   🟡 Yellow Zone (2x Robot Size): %.2f m  -> 2x SLOWDOWN", yellow_dist_);
  RCLCPP_INFO(this->get_logger(), "   🟢 Green Zone:                  > %.2f m -> FULL SPEED (Auto-Resume)", yellow_dist_);
  RCLCPP_INFO(this->get_logger(), "============================================================");
}

void SafetyZoneControllerCpp::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double closest_dist = std::numeric_limits<double>::infinity();
  double angle = msg->angle_min;
  double half_fov = fov_rad_ / 2.0;

  for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
    double r = msg->ranges[i];
    // Safeguard: Explicit finite & range sanity checks
    if (!std::isfinite(r) || std::isnan(r) || std::isinf(r)) {
      continue;
    }
    if (r < msg->range_min || r > msg->range_max) {
      continue;
    }

    // Wrap angle to [-pi, pi]
    double norm_angle = std::atan2(std::sin(angle), std::cos(angle));

    // Calculate obstacle point coordinates in robot base_footprint frame
    double px = lidar_x_offset_ + r * std::cos(norm_angle);
    double py = r * std::sin(norm_angle);
    double d_robot = std::sqrt(px * px + py * py);
    double angle_from_base = std::atan2(py, px);

    // Check if within forward Field of View relative to robot base
    if (px > 0.0 && std::abs(angle_from_base) <= half_fov) {
      if (d_robot < closest_dist) {
        closest_dist = d_robot;
      }
    }
  }

  min_obstacle_distance_ = closest_dist;
}

void SafetyZoneControllerCpp::safetyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    manual_safety_active_ = true;
    has_fresh_raw_cmd_ = false;
    RCLCPP_WARN(this->get_logger(), "🛑 Safety ACTIVE (/safety_stop: true) -> Robot stopped & movement blocked!");
  } else {
    manual_safety_active_ = false;
    has_fresh_raw_cmd_ = false;
    RCLCPP_INFO(this->get_logger(), "🟢 Safety INACTIVE (/safety_stop: false) -> Movement unlocked. Waiting for fresh /cmd_vel_raw.");
  }
}

void SafetyZoneControllerCpp::cmdRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  raw_cmd_ = *msg;
  last_cmd_time_ = this->now();
  has_fresh_raw_cmd_ = true;
}

void SafetyZoneControllerCpp::controlLoop()
{
  // 1. Evaluate safety zone state
  std::string previous_state = zone_state_;
  if (min_obstacle_distance_ <= red_dist_) {
    zone_state_ = "RED";
  } else if (min_obstacle_distance_ <= yellow_dist_) {
    zone_state_ = "YELLOW";
  } else {
    zone_state_ = "GREEN";
  }

  if (zone_state_ != previous_state) {
    if (zone_state_ == "RED") {
      RCLCPP_WARN(this->get_logger(), "🔴 RED ZONE: Obstacle at %.2f m -> EMERGENCY STOP ACTIVATED!", min_obstacle_distance_);
    } else if (zone_state_ == "YELLOW") {
      RCLCPP_INFO(this->get_logger(), "🟡 YELLOW ZONE: Obstacle at %.2f m -> Velocity reduced by 2x.", min_obstacle_distance_);
    } else {
      RCLCPP_INFO(this->get_logger(), "🟢 GREEN ZONE: Path Clear (%.2f m) -> Full speed restored.", min_obstacle_distance_);
    }
  }

  // 2. Compute safe commanded twist
  geometry_msgs::msg::Twist safe_cmd;

  if (manual_safety_active_ || !has_fresh_raw_cmd_) {
    // Safety protection active OR waiting for fresh velocity after release: output zero
    safe_cmd.linear.x = 0.0;
    safe_cmd.angular.z = 0.0;
  } else {
    // Check watchdog timeout on raw input (0.5s)
    double dt = (this->now() - last_cmd_time_).seconds();
    if (dt < 0.5) {
      if (zone_state_ == "RED") {
        // Forward motion locked. Allow reverse and in-place rotation for escape
        if (raw_cmd_.linear.x > 0.0) {
          safe_cmd.linear.x = 0.0;
        } else {
          safe_cmd.linear.x = raw_cmd_.linear.x * slowdown_factor_;
        }
        safe_cmd.angular.z = raw_cmd_.angular.z * slowdown_factor_;
      } else if (zone_state_ == "YELLOW") {
        safe_cmd.linear.x = raw_cmd_.linear.x * slowdown_factor_;
        safe_cmd.angular.z = raw_cmd_.angular.z;
      } else {
        safe_cmd = raw_cmd_;
      }
    }
  }

  pub_cmd_safe_->publish(safe_cmd);

  // 3. Publish RViz visualization markers
  publishMarkers();
}

void SafetyZoneControllerCpp::publishMarkers()
{
  visualization_msgs::msg::MarkerArray markers;

  // Yellow warning cylinder boundary
  visualization_msgs::msg::Marker yellow_marker;
  yellow_marker.header.frame_id = "base_footprint";
  yellow_marker.header.stamp = rclcpp::Time(0);
  yellow_marker.ns = "safety_zones";
  yellow_marker.id = 0;
  yellow_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  yellow_marker.action = visualization_msgs::msg::Marker::ADD;
  yellow_marker.pose.position.x = 0.0;
  yellow_marker.pose.position.y = 0.0;
  yellow_marker.pose.position.z = 0.005;
  yellow_marker.pose.orientation.w = 1.0;
  yellow_marker.scale.x = yellow_dist_ * 2.0;
  yellow_marker.scale.y = yellow_dist_ * 2.0;
  yellow_marker.scale.z = 0.01;
  yellow_marker.color.r = 1.0f;
  yellow_marker.color.g = 0.85f;
  yellow_marker.color.b = 0.0f;
  yellow_marker.color.a = (zone_state_ == "YELLOW") ? 0.35f : 0.12f;
  markers.markers.push_back(yellow_marker);

  // Red emergency stop cylinder boundary
  visualization_msgs::msg::Marker red_marker;
  red_marker.header.frame_id = "base_footprint";
  red_marker.header.stamp = rclcpp::Time(0);
  red_marker.ns = "safety_zones";
  red_marker.id = 1;
  red_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  red_marker.action = visualization_msgs::msg::Marker::ADD;
  red_marker.pose.position.x = 0.0;
  red_marker.pose.position.y = 0.0;
  red_marker.pose.position.z = 0.01;
  red_marker.pose.orientation.w = 1.0;
  red_marker.scale.x = red_dist_ * 2.0;
  red_marker.scale.y = red_dist_ * 2.0;
  red_marker.scale.z = 0.015;
  red_marker.color.r = 1.0f;
  red_marker.color.g = 0.0f;
  red_marker.color.b = 0.0f;
  red_marker.color.a = (zone_state_ == "RED") ? 0.45f : 0.15f;
  markers.markers.push_back(red_marker);

  pub_markers_->publish(markers);
}

}  // namespace mobile_robot_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mobile_robot_bringup::SafetyZoneControllerCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### Step 4.5: Create Python Safety Zone Implementation (`scripts/safety_zone_controller.py`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/scripts/safety_zone_controller.py
chmod +x ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/scripts/safety_zone_controller.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/scripts/safety_zone_controller.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Mobile Robot Safety Zone Controller (LiDAR Collision Avoidance)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Safety Rules:
 1. Red Zone (1x Robot Size ~ 0.45m):
    - Robot stops completely when an obstacle is within 0.45m in the forward path.
    - Publishes True to /safety_stop.
 2. Yellow Zone (2x Robot Size ~ 0.90m):
    - Robot slows down by 2x (0.5x linear speed) when obstacle is between 0.45m and 0.90m.
    - Publishes False to /safety_stop so it does NOT lock up.
 3. Green Zone (> 0.90m):
    - Full normal speed (1.0x).
    - Automatically restores full speed as soon as obstacle moves away!
 4. Clean Steady RViz Display:
    - Displays clean Red (0.45m) and Yellow (0.90m) zone circles without blinking.
================================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class SafetyZoneController(Node):
    def __init__(self):
        super().__init__('safety_zone_controller')

        # ----------------------------------------------------------------------
        # 1. Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('red_zone_distance', 0.45)     # 1x robot size: Stop
        self.declare_parameter('yellow_zone_distance', 0.90)  # 2x robot size: 2x slowdown
        self.declare_parameter('fov_angle_deg', 90.0)         # Forward detection (+/- 45 deg)
        self.declare_parameter('slowdown_factor', 0.5)
        self.declare_parameter('lidar_x_offset', 0.10)        # LiDAR mounting offset in base frame

        self.red_dist = self.get_parameter('red_zone_distance').get_parameter_value().double_value
        self.yellow_dist = self.get_parameter('yellow_zone_distance').get_parameter_value().double_value
        self.fov_rad = math.radians(self.get_parameter('fov_angle_deg').get_parameter_value().double_value)
        self.slowdown = self.get_parameter('slowdown_factor').get_parameter_value().double_value
        self.lidar_x_offset = self.get_parameter('lidar_x_offset').get_parameter_value().double_value

        # State
        self.min_obstacle_distance = float('inf')
        self.zone_state = "GREEN"  # "GREEN", "YELLOW", "RED"
        self.raw_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.manual_safety_active = False
        self.has_fresh_raw_cmd = False

        # Raw command from Twist Mux or Joystick
        self.sub_cmd_raw = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_raw_callback,
            10
        )
        self.sub_joy = self.create_subscription(
            Twist,
            '/joy_vel',
            self.cmd_raw_callback,
            10
        )

        # Safety stop subscription (true: activate safety, false: deactivate safety)
        self.sub_safety_stop = self.create_subscription(
            Bool,
            '/safety_stop',
            self.safety_stop_callback,
            10
        )

        # LiDAR scan (SensorDataQoS for compatibility with both Gazebo and physical hardware)
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Filtered safe command to Diff Drive Controller
        self.pub_cmd_safe = self.create_publisher(Twist, '/cmd_vel', 10)

        # Clean zone boundary circles for RViz
        self.pub_markers = self.create_publisher(MarkerArray, '/safety_zone_markers', 10)

        # Control loop at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🛡️  Safety Zone Controller Initialized')
        self.get_logger().info(f'   🔴 Red Zone (1x Robot Size):    {self.red_dist:.2f} m  -> FULL STOP')
        self.get_logger().info(f'   🟡 Yellow Zone (2x Robot Size): {self.yellow_dist:.2f} m  -> 2x SLOWDOWN')
        self.get_logger().info(f'   🟢 Green Zone:                  > {self.yellow_dist:.2f} m -> FULL SPEED (Auto-Resume)')
        self.get_logger().info('=' * 60)

    def scan_callback(self, msg: LaserScan):
        """Measures closest obstacle in forward field of view relative to robot base."""
        half_fov = self.fov_rad / 2.0
        valid_distances = []
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r) and not math.isnan(r) and not math.isinf(r) and (msg.range_min <= r <= msg.range_max):
                norm_angle = math.atan2(math.sin(angle), math.cos(angle))
                # Compute point coordinates relative to robot base_footprint (0,0)
                px = self.lidar_x_offset + r * math.cos(norm_angle)
                py = r * math.sin(norm_angle)
                d_robot = math.hypot(px, py)
                angle_from_base = math.atan2(py, px)

                if px > 0.0 and abs(angle_from_base) <= half_fov:
                    valid_distances.append(d_robot)
            angle += msg.angle_increment

        if valid_distances:
            self.min_obstacle_distance = min(valid_distances)
        else:
            self.min_obstacle_distance = float('inf')

        # Automatically determine active zone
        if self.min_obstacle_distance <= self.red_dist:
            self.zone_state = "RED"
        elif self.min_obstacle_distance <= self.yellow_dist:
            self.zone_state = "YELLOW"
        else:
            self.zone_state = "GREEN"

    def safety_stop_callback(self, msg: Bool):
        if msg.data:
            self.manual_safety_active = True
            self.has_fresh_raw_cmd = False
            self.get_logger().warn('🛑 Safety ACTIVE (/safety_stop: true) -> Robot stopped & movement blocked!')
        else:
            self.manual_safety_active = False
            self.has_fresh_raw_cmd = False
            self.get_logger().info('🟢 Safety INACTIVE (/safety_stop: false) -> Movement unlocked. Waiting for fresh /cmd_vel_raw.')

    def cmd_raw_callback(self, msg: Twist):
        self.raw_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.has_fresh_raw_cmd = True

    def control_loop(self):
        safe_cmd = Twist()

        if self.manual_safety_active or not self.has_fresh_raw_cmd:
            # Safety protection active OR waiting for fresh velocity after release: output zero
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0
        else:
            # 1. Command Timeout Watchdog: if no command received for > 0.5s, clear command
            cmd_age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if cmd_age <= 0.5:
                if self.zone_state == "RED":
                    # Red Zone: Obstacle is within 1x robot size -> FULL STOP
                    # Prevent moving forward into obstacle; allow reversing away
                    if self.raw_cmd.linear.x > 0.0:
                        safe_cmd.linear.x = 0.0
                    else:
                        safe_cmd.linear.x = self.raw_cmd.linear.x * self.slowdown
                    safe_cmd.angular.z = self.raw_cmd.angular.z * self.slowdown
                elif self.zone_state == "YELLOW":
                    # Yellow Zone: Obstacle is within 2x robot size -> 2x SLOWDOWN
                    safe_cmd.linear.x = self.raw_cmd.linear.x * self.slowdown
                    safe_cmd.angular.z = self.raw_cmd.angular.z
                else:
                    # Green Zone: Clear -> FULL SPEED
                    safe_cmd.linear.x = self.raw_cmd.linear.x
                    safe_cmd.angular.z = self.raw_cmd.angular.z

        self.pub_cmd_safe.publish(safe_cmd)

        # Publish clean visual zone circles to RViz
        self.publish_zone_circles()

    def publish_zone_circles(self):
        """Publishes steady, non-blinking circular boundary rings for Red & Yellow zones."""
        marker_array = MarkerArray()
        # Using stamp 0 (Time().to_msg()) tells RViz to always use the latest TF frame,
        # completely preventing "message too old" / "extrapolation error" blinking!
        zero_stamp = Time().to_msg()

        # 1. Red Zone Marker (Radius = 0.45m, 1x Robot Size)
        red_marker = Marker()
        red_marker.header.frame_id = 'base_footprint'
        red_marker.header.stamp = zero_stamp
        red_marker.ns = 'safety_zones'
        red_marker.id = 1
        red_marker.type = Marker.CYLINDER
        red_marker.action = Marker.ADD
        red_marker.pose.position.x = 0.0
        red_marker.pose.position.y = 0.0
        red_marker.pose.position.z = 0.005
        red_marker.pose.orientation.w = 1.0
        red_marker.scale.x = self.red_dist * 2.0
        red_marker.scale.y = self.red_dist * 2.0
        red_marker.scale.z = 0.005
        red_marker.color.r = 1.0
        red_marker.color.g = 0.05
        red_marker.color.b = 0.05
        red_marker.color.a = 0.35 if self.zone_state == "RED" else 0.18
        marker_array.markers.append(red_marker)

        # 2. Yellow Zone Marker (Radius = 0.90m, 2x Robot Size)
        yellow_marker = Marker()
        yellow_marker.header.frame_id = 'base_footprint'
        yellow_marker.header.stamp = zero_stamp
        yellow_marker.ns = 'safety_zones'
        yellow_marker.id = 2
        yellow_marker.type = Marker.CYLINDER
        yellow_marker.action = Marker.ADD
        yellow_marker.pose.position.x = 0.0
        yellow_marker.pose.position.y = 0.0
        yellow_marker.pose.position.z = 0.002
        yellow_marker.pose.orientation.w = 1.0
        yellow_marker.scale.x = self.yellow_dist * 2.0
        yellow_marker.scale.y = self.yellow_dist * 2.0
        yellow_marker.scale.z = 0.003
        yellow_marker.color.r = 1.0
        yellow_marker.color.g = 0.85
        yellow_marker.color.b = 0.0
        yellow_marker.color.a = 0.30 if self.zone_state == "YELLOW" else 0.12
        marker_array.markers.append(yellow_marker)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyZoneController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
```

### Step 4.6: Create Safety Zone Launcher (`launch/safety_zone.launch.py`)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/launch/safety_zone.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/launch/safety_zone.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Mobile Robot Safety Zone Launcher
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Launches the Safety Zone Controller (Active LiDAR collision avoidance).
Supports switching between C++ binary (default) and Python script via use_cpp:=true/false.
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Launch C++ safety zone node if true, else Python node'
    )

    # C++ Safety Zone Node
    safety_cpp_node = Node(
        package='mobile_robot_bringup',
        executable='safety_zone_controller_cpp',
        name='safety_zone_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_cpp'))
    )

    # Python Safety Zone Node
    safety_py_node = Node(
        package='mobile_robot_bringup',
        executable='safety_zone_controller.py',
        name='safety_zone_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=UnlessCondition(LaunchConfiguration('use_cpp'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_cpp_arg,
        safety_cpp_node,
        safety_py_node
    ])
```

### Step 4.7: Edit `package.xml` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/package.xml
```
*Paste and save:*
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mobile_robot_bringup</name>
  <version>1.0.0</version>
  <description>Complete system bringup launch files (Gazebo, Joystick Deadman, Twist Mux, Safety Zone C++ &amp; Python) for 4-wheel robot.</description>
  <maintainer email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</maintainer>
  <author email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</author>
  <license>Apache-2.0</license>

  <url type="website">https://rahul-r-joshua.github.io/portfolio/</url>
  <url type="repository">https://github.com/rahul-r-joshua</url>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>visualization_msgs</depend>

  <exec_depend>mobile_robot_description</exec_depend>
  <exec_depend>mobile_robot_controller</exec_depend>
  <exec_depend>mobile_robot_firmware</exec_depend>
  <exec_depend>teleop_twist_joy</exec_depend>
  <exec_depend>twist_mux</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 4.8: Edit `CMakeLists.txt` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/CMakeLists.txt
```
*Paste and save:*
```cmake
cmake_minimum_required(VERSION 3.8)
project(mobile_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 1. C++ Safety Zone Node
add_executable(safety_zone_controller_cpp src/safety_zone_controller.cpp)
target_include_directories(safety_zone_controller_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(safety_zone_controller_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

# Install C++ binary
install(TARGETS
  safety_zone_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include
)

# 2. Python Script
install(PROGRAMS
  scripts/safety_zone_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install Launch and Config directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### Step 4.9: Build `mobile_robot_bringup`
```bash
cd ~/ros2_mobile_robot_ws
colcon build --packages-select mobile_robot_bringup --symlink-install
source install/setup.bash
```

---

### ⚡ MILESTONE 4 VERIFICATION (Active Collision Avoidance Execution)

> [!IMPORTANT]
> ### ⚠️ Beginner Rule: Environment Sourcing
> In **EVERY** new terminal tab you open throughout this verification, always execute:
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_mobile_robot_ws/install/setup.bash
> ```

> [!NOTE]
> ### 🚀 Strict Launch Sequence
> **1st: Gazebo** ➡️ **2nd: Controller** ➡️ **3rd: Display / RViz** ➡️ **4th: Safety Zone Controller** ➡️ **5th: Joy (Joystick Teleop)**  
>
> 1. **Gazebo** initializes simulation physics and publishes `/clock`.  
> 2. **Controller** activates `joint_state_broadcaster` and `diff_drive_controller`, broadcasting the dynamic `odom -> base_footprint` transform.  
> 3. **Display / RViz** connects to the simulation clock and renders the robot model, live LiDAR points, and safety zones.  
> 4. **Safety Zone Controller** intercepts `/scan` to evaluate obstacle proximity and control the `/safety_stop` latch.  
> 5. **Joy** launches gamepad drivers, Deadman switch, and velocity multiplexing.

> [!TIP]
> ### 💡 Clean Architecture: Why Zero Remappings Are Needed
> In traditional tutorials, developers often remapped node outputs to daisy-chain velocity topics. Our modular architecture is clean, decoupled, and direct:  
> * `ros2_controller.launch.py` manages wheel actuators and joint state publishing.  
> * `display.launch.py` connects to simulation and visualizes robot kinematics, scan data, and safety zones.  
> * `safety_zone_controller` monitors LiDAR `/scan` and controls `/safety_stop` and speed reduction.  
> * `joystick_teleop.launch.py` reads `/joy` and publishes `/joy_vel` (priority 99 in `twist_mux`).  
> * `twist_mux` routes `/joy_vel` into `/cmd_vel` without requiring manual CLI remappings!

#### Terminal 1: Launch Simulation Engine (1st: Gazebo / Ignition / Gz)

* **Option A: Gazebo Classic (Gazebo 11):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
source install/setup.bash
ros2 launch mobile_robot_description gazebo.launch.py
```

* **Option B: Ignition Gazebo (Fortress):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_description ign_gazebo.launch.py
```

* **Option C: Modern Gazebo (Gz Sim / Harmonic / Garden):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_description ign_gazebo.launch.py backend:=gz
```

#### Terminal 2: Launch Controller (2nd: Controller)
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_controller ros2_controller.launch.py
```

#### Terminal 3: Launch RViz Visualization (3rd: Display / RViz)
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_description display.launch.py
```

> [!TIP]
> ### 🎛️ Automatic RViz Profile Configuration
> In Milestone 4, `simulation.rviz` is loaded with **`Fixed Frame: odom`**!  
> In this configuration:
> * **Fixed Frame** is automatically locked to `odom`.
> * **LaserScan** (`/scan`), **Odometry** (`/odom`), **RobotModel**, and **Safety Zones (Red & Yellow marker rings)** are **pre-configured and visible immediately** without needing to manually add displays in the GUI!
>
> 💡 **Manual Marker Addition Note (If starting from an empty RViz window):**  
> If you are configuring displays in an empty RViz window from scratch, click **Add** (bottom left) ➔ Select **By topic** tab ➔ Choose `/safety_zone_markers` ➔ Select `MarkerArray` ➔ Click **OK**. The Red ($0.45\text{m}$) and Yellow ($0.90\text{m}$) circular safety zone rings will immediately appear around the robot!

#### Terminal 4: Launch Safety Zone Controller (4th: Safety Zone)

* **Option A: Launch C++ Safety Zone Controller Node (`rclcpp`):**
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_bringup safety_zone.launch.py use_cpp:=true
```

* **Option B: Launch Python Safety Zone Controller Node (`rclpy`):**
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_bringup safety_zone.launch.py use_cpp:=false
```

#### Terminal 5: Launch Joystick Teleop (5th: Joy)
```bash
cd ~/ros2_mobile_robot_ws
source install/setup.bash
ros2 launch mobile_robot_controller joystick_teleop.launch.py
```

#### Terminal 6: Test Driving & Obstacle Collision Interception

> [!TIP]
> 🔴 **Spawning an Obstacle in Gazebo for Collision Testing:**  
> In an empty Gazebo world, LiDAR rays travel out to infinity without returning any obstacle points.  
> 1. In the **Gazebo top toolbar**, click the **Cube (Box)** or **Cylinder** icon.  
> 2. Click on the ground plane in front of the robot (approx. $1.5\text{m}$ ahead).  
> 3. In **RViz**, bright red LiDAR points from `/scan` will immediately outline the object surface.  
> 4. Drive towards the object: observe the automatic speed reduction in the Yellow Zone ($< 0.90\text{m}$) and the complete emergency stop lock in the Red Zone ($< 0.45\text{m}$).

##### Option A: Using Physical Gamepad / Joystick Controller
* **Obstacle Interception Test:**
  * Hold **LB (Button 4: Deadman Switch)** and push the **Left Stick forward** towards an obstacle:  
    * As the obstacle enters the **Yellow Zone** ($0.90\text{m}$), forward speed is automatically reduced by 50% for safe approach!  
    * As the obstacle enters the **Red Zone** ($0.45\text{m}$), the safety controller asserts `/safety_stop: true` on `twist_mux`, completely stopping the robot!  
  * **Pull the stick backward** while holding LB to safely reverse away from the obstacle. Speed and control automatically restore!

##### Option B: Using Keyboard Teleoperation (No Gamepad Required)
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw
```

##### Option C: Using Terminal Driving Commands (No Gamepad Required)
* **1. Drive Forward toward Obstacle (Press Ctrl+C to stop):**
```bash
ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

* **2. Trigger Emergency Stop Lock:**
```bash
ros2 topic pub --once /safety_stop std_msgs/msg/Bool "data: true"
```

* **3. Release Emergency Stop (Auto-resumes safely):**
```bash
ros2 topic pub --once /safety_stop std_msgs/msg/Bool "data: false"
```

* **4. Echo Filtered Velocity Output (`/cmd_vel`):**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /cmd_vel
```

---

<a id="milestone-5" name="milestone-5"></a>
## Milestone 5
### Master Simulation Bringup (`mobile_robot_bringup` - `simulate_robot.launch.py`)

Now we assemble the complete simulation bringup launch file that starts Gazebo, Controllers, Twist Mux, Safety Zone, and RViz together using staged timer delays (avoiding race conditions and startup errors).

### Step 5.1: Create `launch/simulate_robot.launch.py` (Staged Timers Architecture)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/launch/simulate_robot.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/launch/simulate_robot.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Simulate Robot Bringup Launch File (Sequential Staged Startup)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Staged Timer Execution (prevents race conditions and RViz TF startup errors):
  1. T = 0.0s: Gazebo Physics & Robot Description (spawn robot on ground)
  2. T = 3.0s: Controller Spawner (ros2_control joint_state_broadcaster & diff_drive)
  3. T = 4.5s: Twist Mux & Joystick Teleop (/joy_vel, /cmd_vel_raw routing)
  4. T = 5.5s: Safety Zone Controller (LiDAR active collision avoidance)
  5. T = 6.5s: RViz2 (Visualizer starts with stable TF tree & active topics)
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory('mobile_robot_description')
    ctrl_pkg = get_package_share_directory('mobile_robot_controller')
    bringup_pkg = get_package_share_directory('mobile_robot_bringup')

    rviz_config_path = os.path.join(desc_pkg, 'rviz', 'simulation.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control diff_drive (true) or custom C++/Python kinematics node (false)'
    )

    use_twist_mux_arg = DeclareLaunchArgument(
        'use_twist_mux',
        default_value='true',
        description='Enable Twist Mux velocity multiplexer and safety stop lock'
    )

    gazebo_backend_arg = DeclareLaunchArgument(
        'gazebo_backend',
        default_value='classic',
        description='Simulator backend: "classic" (Gazebo 11), "ign" (Ignition Fortress), or "gz" (Modern Gazebo)'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Use C++ safety zone node (true) or Python safety zone script (false)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 visualizer (set false for headless/low-resource mode)'
    )

    # -------------------------------------------------------------------------
    # STAGE 1 (T = 0.0s): Gazebo Physics Simulation & Robot Description
    # -------------------------------------------------------------------------
    gazebo_classic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_ros2_control': LaunchConfiguration('use_ros2_control')
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('gazebo_backend'), "' == 'classic'"])
        )
    )

    gazebo_ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'ign_gazebo.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('gazebo_backend'), "' in ['ign', 'gz']"])
        )
    )

    # -------------------------------------------------------------------------
    # STAGE 2 (T = 3.0s): Controllers (ros2_control or Custom Kinematics)
    # -------------------------------------------------------------------------
    ros2_controller_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'ros2_controller.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('use_ros2_control'))
    )

    custom_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'controller.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_cpp': LaunchConfiguration('use_cpp')
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_ros2_control'))
    )

    stage2_controllers = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg=">>> [STAGE 2] Spawning Controllers..."),
            ros2_controller_spawner,
            custom_controller_launch
        ]
    )

    # -------------------------------------------------------------------------
    # STAGE 3 (T = 4.5s): Twist Mux & Joystick Teleop
    # -------------------------------------------------------------------------
    joystick_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'joystick_teleop.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_out': 'cmd_vel_raw'
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_twist_mux'))
    )

    stage3_teleop = TimerAction(
        period=4.5,
        actions=[
            LogInfo(msg=">>> [STAGE 3] Launching Twist Mux & Teleop Stack..."),
            joystick_teleop_launch
        ]
    )

    # -------------------------------------------------------------------------
    # STAGE 4 (T = 5.5s): Safety Zone Controller (Active LiDAR Collision Avoidance)
    # -------------------------------------------------------------------------
    safety_zone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'safety_zone.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_cpp': LaunchConfiguration('use_cpp')
        }.items()
    )

    stage4_safety = TimerAction(
        period=5.5,
        actions=[
            LogInfo(msg=">>> [STAGE 4] Activating Safety Zone Collision Avoidance..."),
            safety_zone_launch
        ]
    )

    # -------------------------------------------------------------------------
    # STAGE 5 (T = 6.5s): RViz2 Visualization (Launched when TF is live)
    # -------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    stage5_rviz = TimerAction(
        period=6.5,
        actions=[
            LogInfo(msg=">>> [STAGE 5] Launching RViz2 Visualizer..."),
            rviz_node
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_ros2_control_arg,
        use_twist_mux_arg,
        gazebo_backend_arg,
        use_cpp_arg,
        use_rviz_arg,
        gazebo_classic_launch,
        gazebo_ign_launch,
        stage2_controllers,
        stage3_teleop,
        stage4_safety,
        stage5_rviz
    ])
```

---

### ⚡ MILESTONE 5 VERIFICATION (Single Command Master Bringup)

> [!IMPORTANT]
> ### ⚠️ Beginner Rule: Environment Sourcing
> In **EVERY** new terminal tab you open throughout this verification, always execute:
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_mobile_robot_ws/install/setup.bash
> ```

> [!NOTE]
> ### 🚀 Single Command Architecture: Autonomous Staged Startup
> In Milestones 1–4, you launched each subsystem manually in separate terminals. In Milestone 5, `simulate_robot.launch.py` brings up the **entire robotics stack** automatically with calibrated `TimerAction` stages:
> 
> ⏱️ **Sequential Execution Stages:**
> 1. **$T = 0.0\text{s}$ (Stage 1):** Physics engine (Gazebo Classic or Ignition/Gz) starts and loads `/robot_description`.
> 2. **$T = 3.0\text{s}$ (Stage 2):** Controllers spawn (`joint_state_broadcaster` and `diff_drive_controller`), establishing the live `odom -> base_footprint` transform tree.
> 3. **$T = 4.5\text{s}$ (Stage 3):** `twist_mux` and joystick teleop engage to route velocity commands safely.
> 4. **$T = 5.5\text{s}$ (Stage 4):** Active LiDAR safety zone controller activates and publishes circular zone markers.
> 5. **$T = 6.5\text{s}$ (Stage 5):** RViz2 visualizer opens with pre-loaded `simulation.rviz` without TF lookup errors or missing displays!

#### Step 5.0: Build `mobile_robot_bringup`
```bash
cd ~/ros2_mobile_robot_ws
colcon build --packages-select mobile_robot_bringup --symlink-install
source install/setup.bash
```

#### Terminal 1: Launch Master Simulation & Robot Bringup (Choose Backend & Mode)

* **Option A: Gazebo Classic (Default Physics + RViz Visualizer):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
source install/setup.bash
ros2 launch mobile_robot_bringup simulate_robot.launch.py
```

* **Option B: Ignition Gazebo (Fortress) Physics Backend:**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_bringup simulate_robot.launch.py gazebo_backend:=ign
```

* **Option C: Modern Gazebo (Gz Sim / Harmonic / Garden) Physics Backend:**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_bringup simulate_robot.launch.py gazebo_backend:=gz
```

* **Option D: Headless Mode (No GUI / Low CPU & RAM for VMs / SSH):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
source install/setup.bash
ros2 launch mobile_robot_bringup simulate_robot.launch.py use_rviz:=false
```

#### Terminal 2: Test Teleoperation & Robot Motion

> [!TIP]
> 🔴 **Seeing LaserScan Points & Testing Obstacle Avoidance:**  
> By default, Gazebo spawns an empty ground plane where LiDAR rays travel out to infinity (`inf`).  
> * To visualize active laser rays and test collision avoidance:  
>   1. In the **Gazebo top toolbar**, click the **Cube (Box)** or **Cylinder** icon.  
>   2. Click on the ground plane in front of the robot ($1.0\text{m}$ to $2.0\text{m}$ ahead).  
>   3. In **RViz**, bright red point hits from `/scan` will outline the obstacle immediately.  
>   4. Drive the robot toward the obstacle using any teleop method below to observe autonomous speed reduction and emergency braking!

##### Option A: Using Physical Gamepad / Joystick Controller (Auto-detected on `/joy`)
* **Normal Mode (0.3 m/s):** Hold **LB (Button 4: Deadman Switch)** and push the **Left Analog Stick** forward to drive, and **Right Analog Stick** left/right to steer.
* **Turbo Mode (0.6 m/s - 2x Speed):** Hold **RB (Button 5: Turbo Switch)** while deflecting the stick for high-speed indoor driving.
* **Deadman Failsafe:** Release the button at any time to instantly stop the robot.

##### Option B: Using Keyboard Teleop (`twist_mux` Key Priority)
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key
```

##### Option C: CLI Velocity Stream (Continuous Test - No Joystick Required)
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub -r 10 /joy_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```
*(Press `Ctrl+C` to stop streaming velocity commands).*

#### Terminal 3: System Health Diagnostics & Controller Inspection

* **Check Active Controllers:**
```bash
source /opt/ros/humble/setup.bash
ros2 control list_controllers
```
*Expected output:* `joint_state_broadcaster[active]` and `diff_drive_controller[active]`.

* **Inspect Active Simulation Topics:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```
*Expected output includes:* `/cmd_vel`, `/joy_vel`, `/odom`, `/scan`, `/camera/image_raw`, `/safety_stop`, `/safety_zone_markers`.

* **Verify Live Coordinate Frame Transformations:**
```bash
source /opt/ros/humble/setup.bash
timeout 5s ros2 run tf2_ros tf2_echo odom base_footprint || true
```

---

<a id="troubleshooting-simulation" name="troubleshooting-simulation"></a>
## Troubleshooting Simulation
### 🛠️ Track 1: Simulation Troubleshooting & Debugging Guide

When working with Gazebo Classic, Ignition Gazebo, and ROS 2 Control, simulation errors can arise from stale background processes, orphaned FastDDS shared memory segments in `/dev/shm`, socket port locks (`11345`), or clock desynchronization.

### 🧹 The Master Simulation Cleanup Script (`clean_simulation.sh`)

If Gazebo exits with code 255, RViz fails to receive topics, or nodes hang on startup, create and execute the master cleanup script to restore your environment to a pristine state.

#### 1. Create the Cleanup Script
Create `~/ros2_mobile_robot_ws/clean_simulation.sh` using `cat << 'EOF'` or your preferred editor:

* **Step 1.1: Write script content:**
```bash
cat << 'EOF' > ~/ros2_mobile_robot_ws/clean_simulation.sh
#!/usr/bin/env bash
# ==============================================================================
# Master ROS 2 & Gazebo Simulation Cleanup Utility
# ------------------------------------------------------------------------------
# Author: Rahul Ramasamy
# Email: rahul.r.joshua123@gmail.com
# GitHub: https://github.com/rahul-r-joshua
# Portfolio: https://rahul-r-joshua.github.io/portfolio/
#
# Complete teardown of all simulation, controller, and visualization processes:
#  1. Force terminates gzserver, gzclient, ign, gz, rviz2, and ROS 2 launch processes.
#  2. Kills controller_manager, spawners, bridges, and python utility nodes.
#  3. Stops the ROS 2 daemon to clear stale DDS discovery tables.
#  4. Purges orphaned FastDDS shared memory (/dev/shm) blocks & semaphore mutexes.
#  5. Cleans temporary Gazebo IPC sockets (/tmp/gazebo* /tmp/gz*).
#  6. Restarts a clean ROS 2 daemon ready for fresh simulation.
# ==============================================================================

set -e

echo "===================================================================="
echo "🧹 Cleaning ROS 2, Gazebo, RViz & FastDDS Shared Memory Environment"
echo "===================================================================="

echo ">>> [1/5] Terminating simulation and visualization processes..."
killall -9 gzserver gzclient rviz2 robot_state_publisher ruby ign gz parameter_bridge 2>/dev/null || true
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
pkill -9 -f "controller_manager" 2>/dev/null || true
pkill -9 -f "spawner" 2>/dev/null || true
pkill -9 -f "twist_relay" 2>/dev/null || true
pkill -9 -f "joint_state_publisher" 2>/dev/null || true
pkill -9 -f "diff_drive_controller" 2>/dev/null || true
pkill -9 -f "safety_zone_controller" 2>/dev/null || true

echo ">>> [2/5] Stopping ROS 2 daemon..."
ros2 daemon stop 2>/dev/null || true

echo ">>> [3/5] Purging orphaned FastDDS shared memory (/dev/shm)..."
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

echo ">>> [4/5] Removing stale Gazebo IPC sockets (/tmp)..."
rm -rf /tmp/gazebo* /tmp/gz* /tmp/ign* 2>/dev/null || true

echo ">>> [5/5] Restarting clean ROS 2 daemon..."
ros2 daemon start 2>/dev/null || true

echo "===================================================================="
echo "✅ Environment completely cleaned! Ready for fresh simulation launch."
echo "===================================================================="
EOF
```

* **Step 1.2: Grant execution permissions:**
```bash
chmod +x ~/ros2_mobile_robot_ws/scripts/clean_simulation.sh
```

#### 2. Run the Cleanup Script
Whenever you need to reset the simulation environment:
```bash
bash ~/ros2_mobile_robot_ws/scripts/clean_simulation.sh
```

> [!TIP]
> **What `clean_simulation.sh` does under the hood:**  
> 1. Force terminates all background `gzserver`, `gzclient`, `rviz2`, and `controller_manager` processes.  
> 2. Purges orphaned FastDDS shared memory blocks (`fastrtps_*`) and semaphore mutexes from `/dev/shm` to prevent DDS discovery collisions.  
> 3. Cleans stale Gazebo IPC sockets from `/tmp`.  
> 4. Restarts the ROS 2 daemon to refresh the active node/topic discovery table.

---

### Diagnosis & Resolution Table

#### 1. `[ERROR] [gzserver-2]: process has died [pid ..., exit code 255]`
* **Root Cause:** A zombie or orphaned `gzserver` process is already running in the background and holding socket port `11345`. When a new launch starts, the new server attempts to bind to `11345` and crashes immediately with `[Err] [Master.cc:96] EXCEPTION: Unable to start server[bind: Address already in use]`.
* **Resolution:**
  Kill all hidden Gazebo processes holding the socket port:
  ```bash
  killall -9 gzserver gzclient
  ```

#### 2. `Waiting for service /spawn_entity, timeout = 30`
* **Root Cause:** `spawn_entity.py` is waiting for the Gazebo ROS factory service `/spawn_entity` to become available. If `gzserver` crashed on startup (e.g. exit code 255 as in Issue 1), the service will never be created, causing the script to hang for 30 seconds and exit.
* **Resolution:**
  Run `bash ~/ros2_mobile_robot_ws/clean_simulation.sh` and re-run your launch file.

#### 3. `[gazebo_ros2_control]: parser error Couldn't parse parameter override rule: '--param robot_description:=...'`
* **Root Cause:** In ROS 2 Humble, `libgazebo_ros2_control.so` extracts the URDF string from `robot_state_publisher` and passes it to its internal `controller_manager` node via an internal CLI parameter override (`--param robot_description:=...`). If the URDF/XACRO contains XML comments with colons followed by a space (e.g. `<!-- Option B: ros2_control -->`) or ampersands (`&`), the underlying ROS 2 argument parser (`rcl/arguments.c`) misinterprets the text as YAML mapping rules or anchors and throws a parse error, failing to load `controller_manager`.
* **Resolution:**
  Never use colons followed by spaces (`: `) or unescaped ampersands (`&`) inside XML comments in your URDF/XACRO files. Use hyphens (`-`) instead (e.g., `<!-- Option B - ros2_control System Plugin -->`) and use the word `and` instead of `&`.

#### 4. `[ign gazebo-2] Library [...] does not export any plugins. The symbol [IgnitionPluginHook] is missing`
* **Root Cause:** Gazebo Classic plugins (`libgazebo_ros_*.so` and `libgazebo_ros2_control.so`) adhere to Gazebo 11's `gazebo::ModelPlugin` / `gazebo::SensorPlugin` C++ interface. Ignition Gazebo (Fortress/Modern Gz) requires `ignition::gazebo::System` plugins exporting the `IgnitionPluginHook` C symbol. If both plugin types are loaded together without conditional separation, Ignition warns that the symbol is missing.
* **Resolution:**
  Condition your plugins in `mobile_robot_gazebo.xacro` using `<xacro:arg name="is_ignition" default="false"/>`:
  ```xml
  <xacro:unless value="$(arg is_ignition)">
    <!-- Gazebo Classic plugins here -->
  </xacro:unless>
  <xacro:if value="$(arg is_ignition)">
    <!-- Ignition Gazebo systems here -->
  </xacro:if>
  ```
  In `ign_gazebo.launch.py`, pass `'is_ignition:=true'` to xacro.

#### 5. Robot Shaking, Jittering, or Flickering in RViz (TF Glitches)
* **Root Cause 1 - Fixed Frame Misconfiguration:** If RViz's **Fixed Frame** is set to `base_footprint` while driving, the camera centers on the chassis while wheel transforms update, creating an optical vibration illusion.
  * **Fix:** In RViz Displays panel, set **Global Options -> Fixed Frame** to `odom`.
* **Root Cause 2 - Clock Desynchronization:** If simulation time (`use_sim_time:=true`) is not enabled on all nodes, nodes mix system wall time with Gazebo simulation time, generating `TF_OLD_DATA` warnings.
  * **Fix:** Ensure all nodes (RViz, controller, robot_state_publisher) receive `use_sim_time: True`.
* **Root Cause 3 - Duplicate Joint State Publishers:** Running both Gazebo's joint state plugin and `joint_state_publisher_gui` simultaneously results in two distinct nodes broadcasting competing transforms for the wheels.
  * **Fix:** `display.launch.py` now defaults to `gui:=false`. Run with `gui:=true` only when inspecting joints standalone without Gazebo.

#### 6. Robot Spinning in Place or Wheels Slipping in Gazebo
* **Root Cause:** Skid-steer 4-wheel differential drive robots experience friction resistance during in-place turns. If lateral friction (`mu2`) is set too high (e.g. `1.0`), the wheels bind to the ground plane and skip.
* **Resolution:**
  Ensure wheel contact friction parameters in `mobile_robot_gazebo.xacro` are tuned:
  ```xml
  <mu1>0.8</mu1>        <!-- High longitudinal rolling traction -->
  <mu2>0.1</mu2>        <!-- Low lateral slip friction for smooth turning -->
  <fdir1>1 0 0</fdir1>  <!-- Alignment vector along rolling direction -->
  <kp>100000.0</kp>     <!-- Stiff contact surface -->
  <kd>1.0</kd>          <!-- Contact damping -->
  ```

#### 7. Automatic Watchdog Stop & Zero-Velocity Latching
* **Behavior:** When publishing `/cmd_vel` from a terminal and pressing `Ctrl+C`, the robot comes to an immediate, clean stop instead of rolling endlessly.
* **How it works:** Both the `diff_drive_controller` node (Option 1 & 2) and `ros2_control` (Option 1 & 3) implement a watchdog timeout (default `0.5s`). When no new Twist message is received within `cmd_vel_timeout`, target wheel speeds are reset to `0.0` and an explicit zero-velocity command is published to physics.

#### 8. Joystick Gamepad Inactive / Robot Not Responding
* **Root Cause 1 - Deadman Button:** Gamepad teleop enforces a deadman switch (Button 4 / LB). The left analog stick only commands velocity while holding LB.
* **Root Cause 2 - Gamepad Device Mapping:** If your controller is recognized as `/dev/input/js1` instead of `js0`, `joy_node` cannot open the joystick.
  * **Fix:** Check your gamepad device index with `ls -l /dev/input/js*` and specify `device_id:=1` if necessary.

#### 9. `controller_manager` Spawner Timeout or Controllers Inactive
* **Check controller status:**
  ```bash
  ros2 control list_controllers
  ```
  Expected output:
  ```text
  joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
  diff_drive_controller[diff_drive_controller/DiffDriveController] active
  ```

#### 10. `[ERROR] [launch]: Caught exception in launch: invalid condition expression, expected one of [true, 1, false, 0] but got 'fasle'`
* **Error Output:**
  ```text
  [ERROR] [launch]: Caught exception in launch (see debug for traceback): invalid condition expression, expected one of [true, 1, false, 0] but got 'fasle', expanded from '<launch.substitutions.launch_configuration.LaunchConfiguration object>'
  ```
* **Root Cause:** A typo in boolean launch arguments (such as `use_cpp:=fasle` instead of `use_cpp:=false` or `use_ros2_control:=fasle`). ROS 2 launch conditions (`IfCondition` / `UnlessCondition`) strictly parse boolean strings and accept only `['true', '1', 'false', '0']`.
* **Resolution:**
  Correct the typo to `false` or `0`:
  ```bash
  ros2 launch mobile_robot_controller controller.launch.py use_cpp:=false
  ```

#### 11. Obstacle Emergency Stop Triggering Early / Red Circle Visual Mismatch
* **Symptom:** In RViz, the robot emergency stops while the red obstacle points from `/scan` are still outside the red safety zone circle ($0.45\text{m}$).
* **Root Cause:** The LiDAR sensor is physically mounted at $X = +0.10\text{m}$ along `base_link`, whereas RViz safety marker cylinders are centered at `base_footprint` $(0, 0)$. Comparing raw LiDAR range $r \le 0.45\text{m}$ directly causes the trigger to fire at $0.10\text{m} + 0.45\text{m} = 0.55\text{m}$ from the robot center, creating a $10\text{cm}$ visual error.
* **Resolution:**
  Transform all scan points into `base_footprint` origin before thresholding:
  $$p_x = \text{lidar\_x\_offset} + r \cos(\theta), \quad p_y = r \sin(\theta), \quad d_{\text{robot}} = \sqrt{p_x^2 + p_y^2}$$
  Both `safety_zone_controller.cpp` and `safety_zone_controller.py` include this coordinate transformation parameter (`lidar_x_offset:=0.10`).

#### 12. ROS 2 Timer Freezes / Callbacks Never Fire when Running Standalone Nodes
* **Symptom:** When running a node standalone (e.g. `safety_zone_controller.py` or `diff_drive_controller.py`) without Gazebo running, no `/safety_zone_markers` or odometry messages are published.
* **Root Cause:** When `use_sim_time:=true` is set, `self.create_timer()` synchronizes with the ROS simulation clock `/clock`. Without Gazebo publishing `/clock`, the ROS clock stays frozen at time 0.
* **Resolution:**
  When testing nodes standalone without simulation physics, always pass `use_sim_time:=false`:
  ```bash
  ros2 launch mobile_robot_bringup safety_zone.launch.py use_sim_time:=false
  ```

#### 13. `ign_ros2_control` Plugin Renaming in Modern Gazebo Sim
* **Symptom:** `[WARN] [gz_ros2_control]: The ign_ros2_control plugin got renamed to gz_ros2_control.`
* **Resolution:**
  In ROS 2 Humble with modern `ros_gz`, update the URDF hardware interface to `<plugin>gz_ros2_control/GazeboSimSystem</plugin>` or use the unified `libign_ros2_control-system.so` / `gz_ros2_control-system` wrapper provided in `mobile_robot_gazebo.xacro`.

---

# 🔌 TRACK 2: EMBEDDED FIRMWARE & PHYSICAL HARDWARE (Hardware Track)

---

<a id="milestone-6" name="milestone-6"></a>
## Milestone 6
### Microcontroller Firmware & Serial Hardware Bridge (`mobile_robot_firmware`)

In Milestone 6, you connect the ROS 2 software stack to physical microcontrollers (Arduino Uno or ESP32) using our robust, bidirectional serial hardware bridge.

> [!IMPORTANT]
> ### 🔑 Microcontroller USB Serial Port Permissions
> Before communicating with physical Arduino Uno or ESP32 boards via `/dev/ttyUSB0` or `/dev/ttyACM0`, ensure your Linux user belongs to the `dialout` group to prevent `Permission denied` errors:
> ```bash
> sudo usermod -a -G dialout $USER
> ```
> *(Note: You must log out and log back in, or run `newgrp dialout`, for group changes to take effect).*

---

<a id="bill-of-materials" name="bill-of-materials"></a>
## Bill of Materials
### 📦 Physical Robot Bill of Materials (BOM) & Components List

| Component | Specification / Model | Quantity | Purpose in Architecture |
| :--- | :--- | :---: | :--- |
| **SBC (Brain)** | Raspberry Pi 4 Model B (4GB/8GB) or Raspberry Pi 5 | 1 | Runs Ubuntu 22.04 LTS, ROS 2 Humble nodes, Twist Mux, Safety Zone, Navigation stack |
| **Microcontroller** | Arduino Uno R3 (ATmega328P) OR ESP32 DevKit V1 (30-pin WROOM) | 1 | Real-time motor PWM output, hardware interrupt encoder counting, IMU polling at 50 Hz |
| **Chassis Kit** | 4-Wheel Differential/Skid-Steer Robot Chassis Kit | 1 | Physical robot frame, dual acrylic/aluminum decks, motor brackets, battery plate |
| **Drive Motors** | 4x TT Geared DC Motors (6V-12V, 1:48 gear ratio) with Hall Encoders | 4 | Robot propulsion and high-resolution wheel displacement feedback |
| **Motor Driver** | L298N Dual H-Bridge (2A peak) OR TB6612FNG Dual Driver (1.2A continuous) | 1 | High-current bidirectional motor drive powered directly from main battery |
| **2D LiDAR** | Slamtec RPLiDAR (A1M8 / A2M8 / A3 / C1) OR YDLidar (X4 / G4 / T-mini / X2) | 1 | 360-degree obstacle detection, 2D LaserScan safety zones, 10Hz SLAM mapping |
| **Camera Module** | Raspberry Pi CSI Camera (V2 / V3) OR USB 1080p HD Webcam OR **ESP32-CAM** | 1 | Visual feedback, object detection, live camera stream over `/camera/image_raw` (HTTP/RTSP for ESP32-CAM) |
| **IMU Sensor** | MPU-6050 (6-DOF Accelerometer + Gyro, I2C `0x68`) OR Bosch **BNO055** (9-DOF Absolute Orientation, I2C `0x28`/`0x29`) | 1 | Rotational velocity ($\omega_z$), linear acceleration, and drift-free fused orientation telemetry |
| **Level Shifter** | 4-Channel Bidirectional 5V-3.3V Logic Level Converter (for ESP32) | 1 | Safely interfaces 5V Hall encoder signals with 3.3V ESP32 GPIO inputs |
| **Main Battery** | 11.1V 3S LiPo Battery (2200mAh 25C) OR 12V Li-ion Pack (3x 18650) | 1 | Isolated high-current power supply for DC motors and motor driver |
| **Logic Battery** | 5V 3A USB-C Power Bank (10,000mAh) OR DC-DC Buck Converter (5V/5A) | 1 | Regulated clean power for Raspberry Pi, Microcontroller, LiDAR, and Sensors |
| **Decoupling Caps**| 0.1µF (100nF) Ceramic Capacitors + 4.7kΩ Pull-up Resistors | 4 | Snubs inductive motor brush noise and pulls up I2C SDA/SCL lines |
| **Hardware** | USB-A to USB-B (Arduino) / Micro-USB (ESP32), Jumper Wires, Standoffs | 1 Set | Serial communications, star-point ground bus, structural mounting |

---

<a id="wiring-diagrams" name="wiring-diagrams"></a>
## Wiring Diagrams
### 🔌 Hardware Circuit Schematics & Complete Wiring Diagrams

#### Schematic 1: Arduino Uno (With Encoders + MPU6050 IMU + L298N Motor Driver)
```text
+-----------------------------------------------------------------------------------+
|                  ARDUINO UNO WITH ENCODERS + IMU + L298N SCHEMATIC                |
+-----------------------------------------------------------------------------------+

     [11.1V - 12V LiPo Battery]
         + (12V) -------------> [L298N 12V Screw Terminal]
         - (GND) --+----------> [L298N GND Screw Terminal]
                   |
                   +----------> [Arduino GND Pin] (COMMON STAR GROUND)

     [Arduino Uno R3]                            [L298N Dual H-Bridge Driver]
     +-------------------+                       +--------------------------+
     |   Digital Pin 5   | (PWM Left) ---------> | ENA (Left Enable)        |
     |   Digital Pin 7   | (Dir L1) -----------> | IN1                      |
     |   Digital Pin 8   | (Dir L2) -----------> | IN2                      |
     |   Digital Pin 9   | (Dir R1) -----------> | IN3                      |
     |   Digital Pin 10  | (Dir R2) -----------> | IN4                      |
     |   Digital Pin 6   | (PWM Right) --------> | ENB (Right Enable)       |
     |                   |                       |                          |
     |                   |                       | OUT1, OUT2 ---> Left  Motors (Parallel)
     |                   |                       | OUT3, OUT4 ---> Right Motors (Parallel)
     |                   |                       +--------------------------+
     |                   |
     |   Digital Pin 2   | <--- Left Encoder Phase A (INT0 - Hardware Interrupt)
     |   Digital Pin 4   | <--- Left Encoder Phase B
     |   Digital Pin 3   | <--- Right Encoder Phase A (INT1 - Hardware Interrupt)
     |   Digital Pin 11  | <--- Right Encoder Phase B
     |                   |
     |   Analog Pin A4   | <---> MPU-6050 SDA (I2C Data)   [+4.7kΩ Pull-up to 5V]
     |   Analog Pin A5   | <---> MPU-6050 SCL (I2C Clock)  [+4.7kΩ Pull-up to 5V]
     |   5V Output Pin   | ----> MPU-6050 VCC & Encoder Hall Sensors VCC (5V)
     |   GND Pin         | ----> MPU-6050 GND & Encoder GND
     +-------------------+
     
     * Decoupling: Solder one 0.1uF ceramic capacitor directly across each DC motor's terminals.
```

#### Schematic 2: Arduino Uno (Without Encoders / Open-Loop + MPU6050 IMU + L298N)
```text
+-----------------------------------------------------------------------------------+
|               ARDUINO UNO OPEN-LOOP (NO ENCODERS) + IMU + L298N SCHEMATIC         |
+-----------------------------------------------------------------------------------+

     [11.1V - 12V LiPo Battery]
         + (12V) -------------> [L298N 12V Screw Terminal]
         - (GND) --+----------> [L298N GND Screw Terminal]
                   |
                   +----------> [Arduino GND Pin] (COMMON STAR GROUND)

     [Arduino Uno R3]                            [L298N Dual H-Bridge Driver]
     +-------------------+                       +--------------------------+
     |   Digital Pin 5   | (PWM Left) ---------> | ENA (Left Enable)        |
     |   Digital Pin 7   | (Dir L1) -----------> | IN1                      |
     |   Digital Pin 8   | (Dir L2) -----------> | IN2                      |
     |   Digital Pin 9   | (Dir R1) -----------> | IN3                      |
     |   Digital Pin 10  | (Dir R2) -----------> | IN4                      |
     |   Digital Pin 6   | (PWM Right) --------> | ENB (Right Enable)       |
     |                   |                       |                          |
     |                   |                       | OUT1, OUT2 ---> Left  Motors (Parallel)
     |                   |                       | OUT3, OUT4 ---> Right Motors (Parallel)
     |                   |                       +--------------------------+
     |                   |
     |   Analog Pin A4   | <---> MPU-6050 SDA (I2C Data)
     |   Analog Pin A5   | <---> MPU-6050 SCL (I2C Clock)
     |   5V Output Pin   | ----> MPU-6050 VCC
     |   GND Pin         | ----> MPU-6050 GND
     +-------------------+
```

#### Schematic 3: ESP32 (With Encoders + Level Shifter + MPU6050 IMU + Motor Driver)
```text
+-----------------------------------------------------------------------------------+
|         ESP32 (3.3V) WITH ENCODERS + LEVEL SHIFTER + IMU + MOTOR DRIVER           |
+-----------------------------------------------------------------------------------+

     [12V Motor Battery] ---------> [L298N 12V In]
     [Common GND] ----------------> [L298N GND, ESP32 GND, Shifter GND, IMU GND]

     [5V Hall Encoders]                 [Logic Level Shifter]          [ESP32 DevKit V1]
     +----------------------+           +-------------------+          +-----------------+
     | Left Enc Phase A (5V)| --------> | HV1 --------> LV1 | -------> | GPIO 18 (INT)   |
     | Left Enc Phase B (5V)| --------> | HV2 --------> LV2 | -------> | GPIO 19         |
     | Right Enc Phase A(5V)| --------> | HV3 --------> LV3 | -------> | GPIO 32 (INT)   |
     | Right Enc Phase B(5V)| --------> | HV4 --------> LV4 | -------> | GPIO 33         |
     | 5V Supply (From Pi/5V)---------> | HV (High Volt Ref)|          |                 |
     |                      |           | LV (3.3V Ref) <--------------| 3V3 Pin         |
     +----------------------+           +-------------------+          +-----------------+

     [ESP32 DevKit V1]                           [L298N Motor Driver]
     +-------------------+                       +--------------------+
     |   GPIO 25 (LEDC0) | (PWM Left) ---------> | ENA                |
     |   GPIO 26         | (Dir L1) -----------> | IN1                |
     |   GPIO 27         | (Dir L2) -----------> | IN2                |
     |   GPIO 14         | (Dir R1) -----------> | IN3                |
     |   GPIO 13         | (Dir R2) -----------> | IN4                |
     |   GPIO 4  (LEDC1) | (PWM Right) --------> | ENB                |
     |                   |                       |                    |
     |   GPIO 21 (SDA)   | <---> MPU-6050 SDA    | OUT1..OUT4 -> Motors
     |   GPIO 22 (SCL)   | <---> MPU-6050 SCL    +--------------------+
     |   3V3 Output Pin  | ----> MPU-6050 VCC (3.3V Native)
     |   GND Pin         | ----> MPU-6050 GND
     +-------------------+
     
     * Safety Notice: Never connect 5V encoder wires directly to ESP32 pins without the level shifter!
```

#### Schematic 4: ESP32 (Without Encoders / Open-Loop + MPU6050 IMU + Motor Driver)
```text
+-----------------------------------------------------------------------------------+
|               ESP32 OPEN-LOOP (NO ENCODERS) + IMU + MOTOR DRIVER SCHEMATIC        |
+-----------------------------------------------------------------------------------+

     [12V Motor Battery] ---------> [L298N 12V In]
     [Common GND] ----------------> [L298N GND, ESP32 GND, IMU GND]

     [ESP32 DevKit V1]                           [L298N Motor Driver]
     +-------------------+                       +--------------------+
     |   GPIO 25 (LEDC0) | (PWM Left) ---------> | ENA                |
     |   GPIO 26         | (Dir L1) -----------> | IN1                |
     |   GPIO 27         | (Dir L2) -----------> | IN2                |
     |   GPIO 14         | (Dir R1) -----------> | IN3                |
     |   GPIO 13         | (Dir R2) -----------> | IN4                |
     |   GPIO 4  (LEDC1) | (PWM Right) --------> | ENB                |
     |                   |                       |                    |
     |   GPIO 21 (SDA)   | <---> MPU-6050 SDA    | OUT1..OUT4 -> Motors
     |   GPIO 22 (SCL)   | <---> MPU-6050 SCL    +--------------------+
     |   3V3 Output Pin  | ----> MPU-6050 VCC
     |   GND Pin         | ----> MPU-6050 GND
     +-------------------+
```

---

### Step 6.1: Create the Package with `ros2 pkg create`
```bash
cd ~/ros2_mobile_robot_ws/src
ros2 pkg create --build-type ament_cmake mobile_robot_firmware
```

### Step 6.2: Create Subdirectories
```bash
cd ~/ros2_mobile_robot_ws/src/mobile_robot_firmware
mkdir -p scripts launch firmware
```

### Step 6.3: Create `scripts/serial_hardware_bridge.py` (Hardware & Mock Modes)
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/scripts/serial_hardware_bridge.py
chmod +x ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/scripts/serial_hardware_bridge.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/scripts/serial_hardware_bridge.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Mobile Robot Serial Hardware Bridge (Encoders + IMU Telemetry)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Bridges ROS 2 topics with the microcontroller (Arduino Uno / ESP32) over USB Serial.

Capabilities:
 1. Bidirectional Communication:
    - Transmits /wheel_speed_commands (rad/s) converted to motor PWM ("L:<pwm>,R:<pwm>\n").
    - Receives encoder ticks ("E:<left_ticks>,<right_ticks>\n") and publishes to /wheel_encoder_ticks.
    - Receives IMU readings ("I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n") and publishes to /imu/data_raw.
 2. Seamless Mock Fallback:
    - If Arduino/ESP32 is not plugged in, node enters MOCK MODE.
    - In Mock Mode, it simulates encoder counts based on commanded speeds so all
      downstream odometry nodes and RViz plugins function identically!
================================================================================
"""

import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import Imu

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class SerialHardwareBridge(Node):
    def __init__(self):
        super().__init__('serial_hardware_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_wheel_speed', 10.0)  # rad/s corresponding to 255 PWM
        self.declare_parameter('deadband_pwm', 30)       # Minimum PWM to overcome motor stiction
        self.declare_parameter('encoder_cpr', 330)       # Counts per revolution
        self.declare_parameter('wheel_radius', 0.05)     # Meters

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_wheel_speed').get_parameter_value().double_value
        self.deadband = self.get_parameter('deadband_pwm').get_parameter_value().integer_value
        self.encoder_cpr = self.get_parameter('encoder_cpr').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.serial_conn = None
        self.is_connected = False

        # Mock simulation state
        self.mock_left_ticks = 0.0
        self.mock_right_ticks = 0.0
        self.last_left_speed = 0.0
        self.last_right_speed = 0.0
        self.last_mock_time = self.get_clock().now()

        # Connect to Hardware (or fallback to Mock)
        self.connect_serial()

        # Publishers
        self.pub_encoder_ticks = self.create_publisher(Int32MultiArray, '/wheel_encoder_ticks', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Subscriber to wheel speeds from Kinematics Controller
        self.sub_speeds = self.create_subscription(
            Float32MultiArray,
            '/wheel_speed_commands',
            self.wheel_speed_callback,
            10
        )

        # Timer to read serial telemetry or update mock simulation (50 Hz)
        self.timer = self.create_timer(0.02, self.telemetry_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🔌 Serial Hardware Bridge Initialized')
        self.get_logger().info(f'   Target Serial Port: {self.port} @ {self.baudrate} baud')
        self.get_logger().info(f'   Hardware Status:    {"CONNECTED (Arduino/ESP32) ✅" if self.is_connected else "MOCK SIMULATION MODE 🟡"}')
        self.get_logger().info(f'   Published Topics:   /wheel_encoder_ticks, /imu/data_raw')
        self.get_logger().info('=' * 60)

    def connect_serial(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('⚠️  pyserial not available. Running in MOCK mode.')
            self.is_connected = False
            return

        # Check alternative ports if /dev/ttyUSB0 doesn't exist
        port_to_try = self.port
        if not os.path.exists(port_to_try):
            for alt in ['/dev/ttyACM0', '/dev/ttyUSB1', '/dev/ttyACM1']:
                if os.path.exists(alt):
                    port_to_try = alt
                    self.port = alt
                    break

        if not os.path.exists(port_to_try):
            self.get_logger().warn(
                f'⚠️  Port {port_to_try} not found. Running in MOCK mode.\n'
                f'   (Connect Arduino/ESP32 via USB: ls /dev/ttyUSB* /dev/ttyACM*)'
            )
            self.is_connected = False
            return

        try:
            self.serial_conn = serial.Serial(
                port=port_to_try,
                baudrate=self.baudrate,
                timeout=0.05
            )
            time.sleep(1.5)  # Allow bootloader reset
            self.is_connected = True
            self.get_logger().info(f'✅ Connected to Microcontroller on {port_to_try}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to open port {port_to_try}: {e}')
            self.is_connected = False

    def rad_s_to_pwm(self, rad_s: float) -> int:
        if abs(rad_s) < 0.05:
            return 0
        ratio = rad_s / self.max_speed
        pwm = int(ratio * 255.0)
        if pwm > 0:
            pwm = max(self.deadband, min(255, pwm))
        elif pwm < 0:
            pwm = min(-self.deadband, max(-255, pwm))
        return pwm

    def wheel_speed_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return

        left_rad_s = msg.data[0]
        right_rad_s = msg.data[1]

        self.last_left_speed = left_rad_s
        self.last_right_speed = right_rad_s

        left_pwm = self.rad_s_to_pwm(left_rad_s)
        right_pwm = self.rad_s_to_pwm(right_rad_s)

        cmd_string = f"L:{left_pwm},R:{right_pwm}\n"

        if self.is_connected and self.serial_conn:
            try:
                self.serial_conn.write(cmd_string.encode('ascii'))
                self.serial_conn.flush()
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

    def telemetry_loop(self):
        """Reads hardware serial messages or simulates mock encoder telemetry."""
        now = self.get_clock().now()

        if self.is_connected and self.serial_conn:
            # Read incoming lines from Arduino / ESP32
            try:
                while self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('E:'):
                        # Format: E:<left_ticks>,<right_ticks>
                        parts = line[2:].split(',')
                        if len(parts) == 2:
                            msg = Int32MultiArray()
                            msg.data = [int(parts[0]), int(parts[1])]
                            self.pub_encoder_ticks.publish(msg)
                    elif line.startswith('I:'):
                        # Format: I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>
                        parts = line[2:].split(',')
                        if len(parts) == 6:
                            imu_msg = Imu()
                            imu_msg.header.stamp = now.to_msg()
                            imu_msg.header.frame_id = 'imu_link'
                            # Scale raw accelerometer (±2g = 16384 LSB/g, 1g = 9.80665 m/s^2)
                            imu_msg.linear_acceleration.x = (float(parts[0]) / 16384.0) * 9.80665
                            imu_msg.linear_acceleration.y = (float(parts[1]) / 16384.0) * 9.80665
                            imu_msg.linear_acceleration.z = (float(parts[2]) / 16384.0) * 9.80665
                            # Scale raw gyro (±250 dps = 131.0 LSB/(deg/s) -> rad/s)
                            deg_to_rad = 3.14159265 / 180.0
                            imu_msg.angular_velocity.x = (float(parts[3]) / 131.0) * deg_to_rad
                            imu_msg.angular_velocity.y = (float(parts[4]) / 131.0) * deg_to_rad
                            imu_msg.angular_velocity.z = (float(parts[5]) / 131.0) * deg_to_rad
                            self.pub_imu.publish(imu_msg)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
        else:
            # Mock mode: Integrate wheel angular velocities into ticks
            dt = (now - self.last_mock_time).nanoseconds / 1e9
            self.last_mock_time = now

            if dt > 0:
                # ticks = (rad_s * dt / (2 * pi)) * CPR
                two_pi = 6.28318530718
                self.mock_left_ticks += (self.last_left_speed * dt / two_pi) * self.encoder_cpr
                self.mock_right_ticks += (self.last_right_speed * dt / two_pi) * self.encoder_cpr

                msg = Int32MultiArray()
                msg.data = [int(self.mock_left_ticks), int(self.mock_right_ticks)]
                self.pub_encoder_ticks.publish(msg)

                # Mock IMU telemetry (gravity on Z, zero angular rate)
                imu_msg = Imu()
                imu_msg.header.stamp = now.to_msg()
                imu_msg.header.frame_id = 'imu_link'
                imu_msg.linear_acceleration.z = 9.80665
                self.pub_imu.publish(imu_msg)

    def destroy_node(self):
        if self.is_connected and self.serial_conn:
            try:
                self.serial_conn.write(b"L:0,R:0\n")
                self.serial_conn.flush()
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialHardwareBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
```

### Step 6.4: Edit `package.xml` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/package.xml
```
*Paste and save:*
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mobile_robot_firmware</name>
  <version>1.0.0</version>
  <description>Microcontroller firmware sketches and serial hardware communication bridge for 4-wheel robot.</description>
  <maintainer email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</maintainer>
  <author email="rahul.r.joshua123@gmail.com">Rahul Ramasamy</author>
  <license>Apache-2.0</license>

  <url type="website">https://rahul-r-joshua.github.io/portfolio/</url>
  <url type="repository">https://github.com/rahul-r-joshua</url>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 6.5: Edit `CMakeLists.txt` with `gedit`
```bash
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/CMakeLists.txt
```
*Paste and save:*
```cmake
cmake_minimum_required(VERSION 3.8)
project(mobile_robot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/serial_hardware_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch firmware
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### Step 6.6: Create `launch/firmware.launch.py`
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/launch/firmware.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/launch/firmware.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino (e.g. /dev/ttyUSB0 or /dev/ttyACM0)'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    hardware_bridge_node = Node(
        package='mobile_robot_firmware',
        executable='serial_hardware_bridge.py',
        name='serial_hardware_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        hardware_bridge_node
    ])
```

### Step 6.7: Microcontroller Firmware Sketches

We provide 4 dedicated firmware sketches matching your hardware platform:

#### Option A: Arduino Uno with Encoders + IMU (`firmware/arduino_motor_controller.ino`)
*Use when your DC gearmotors have quadrature encoders:*
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/firmware/arduino_motor_controller.ino
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/firmware/arduino_motor_controller.ino
```
*Paste and save:*
```cpp
/**
 * ============================================================================
 * 4-Wheel Differential Drive Robot - Arduino Firmware (Encoders + IMU)
 * ============================================================================
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 * 
 * Workshop Edition for ROS 2 Mobile Robotics Workshop.
 * 
 * Features:
 *  1. Dual H-Bridge Motor Control (L298N / TB6612FNG).
 *  2. DC Motor Quadrature Encoders via Hardware Interrupts (Pins 2 & 3).
 *  3. MPU6050 6-DOF IMU integration via I2C (Pins A4-SDA, A5-SCL).
 *  4. 1.0s Watchdog timer for fail-safe emergency motor stopping.
 *  5. High-speed bidirectional Serial communication with ROS 2 @ 115200 baud.
 * 
 * Serial Protocol:
 *  - Incoming (from ROS 2): "L:<left_pwm>,R:<right_pwm>\n"   (-255 to 255)
 *  - Outgoing (to ROS 2):   "E:<left_ticks>,<right_ticks>\n" (Encoder Feedback)
 *                           "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n" (Raw IMU Feedback)
 * ============================================================================
 */

#include <Wire.h>

// -----------------------------------------------------------------------------
// 1. Motor Pin Definitions (L298N)
// -----------------------------------------------------------------------------
const int ENA = 5;    // Left Motors PWM Speed
const int IN1 = 7;    // Left Direction 1
const int IN2 = 8;    // Left Direction 2

const int ENB = 6;    // Right Motors PWM Speed
const int IN3 = 9;    // Right Direction 1
const int IN4 = 10;   // Right Direction 2

// -----------------------------------------------------------------------------
// 2. Encoder Pin Definitions & Variables
// -----------------------------------------------------------------------------
const int ENC_LEFT_A  = 2;  // External Interrupt INT0
const int ENC_LEFT_B  = 4;  // Direction logic
const int ENC_RIGHT_A = 3;  // External Interrupt INT1
const int ENC_RIGHT_B = 11; // Direction logic

volatile long left_encoder_ticks  = 0;
volatile long right_encoder_ticks = 0;

// -----------------------------------------------------------------------------
// 3. IMU Configuration (Supports both MPU6050 and BNO055)
// -----------------------------------------------------------------------------
const int MPU6050_ADDR = 0x68;
const int BNO055_ADDR  = 0x28;

enum ImuType { IMU_NONE, IMU_MPU6050, IMU_BNO055 };
ImuType active_imu = IMU_NONE;

// -----------------------------------------------------------------------------
// 4. Timing & Watchdog
// -----------------------------------------------------------------------------
const unsigned long TIMEOUT_MS = 1000;    // 1 second safety watchdog
const unsigned long TELEMETRY_MS = 50;   // Send encoder & IMU feedback at 20 Hz
unsigned long last_cmd_time = 0;
unsigned long last_telemetry_time = 0;

// -----------------------------------------------------------------------------
// Interrupt Service Routines (ISRs) for Encoders
// -----------------------------------------------------------------------------
void isrLeftEncoder() {
  if (digitalRead(ENC_LEFT_B) == HIGH) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}

void isrRightEncoder() {
  if (digitalRead(ENC_RIGHT_B) == HIGH) {
    right_encoder_ticks++;
  } else {
    right_encoder_ticks--;
  }
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // Configure Motor Pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Stop motors initially
  stopMotors();

  // Configure Encoder Pins with Internal Pullups
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightEncoder, RISING);

  // Initialize I2C and detect IMU (MPU6050 or BNO055)
  Wire.begin();
  
  // 1. Try MPU6050 (0x68)
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    active_imu = IMU_MPU6050;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission();
  } else {
    // 2. Try Bosch BNO055 (0x28)
    Wire.beginTransmission(BNO055_ADDR);
    if (Wire.endTransmission() == 0) {
      active_imu = IMU_BNO055;
      Wire.beginTransmission(BNO055_ADDR);
      Wire.write(0x3D); // OPR_MODE register
      Wire.write(0x08); // IMU mode
      Wire.endTransmission();
    }
  }

  Serial.println("==================================================");
  Serial.println("✓ 4-Wheel Robot Arduino Firmware Ready!");
  Serial.print("  Encoders: Active on INT0 (Pin 2) & INT1 (Pin 3)\n");
  Serial.print("  IMU: ");
  if (active_imu == IMU_MPU6050) {
    Serial.println("MPU-6050 Detected & Initialized ✓");
  } else if (active_imu == IMU_BNO055) {
    Serial.println("Bosch BNO-055 Detected & Initialized ✓");
  } else {
    Serial.println("None Detected (Skipped)");
  }
  Serial.println("  Safety Watchdog: 1000 ms");
  Serial.println("==================================================");

  last_cmd_time = millis();
  last_telemetry_time = millis();
}

// -----------------------------------------------------------------------------
// Main Loop
// -----------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // 1. Process incoming commands from ROS 2
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      parseAndExecuteCommand(input);
      last_cmd_time = now;
    }
  }

  // 2. Safety Watchdog: Stop motors if ROS 2 freezes or disconnects
  if (now - last_cmd_time > TIMEOUT_MS) {
    stopMotors();
  }

  // 3. Periodic Telemetry (Encoder Ticks & IMU data at 20 Hz)
  if (now - last_telemetry_time >= TELEMETRY_MS) {
    last_telemetry_time = now;
    sendTelemetry();
  }
}

// -----------------------------------------------------------------------------
// Parse ROS 2 Serial Command: "L:<left>,R:<right>"
// -----------------------------------------------------------------------------
void parseAndExecuteCommand(String cmd) {
  int l_index = cmd.indexOf("L:");
  int r_index = cmd.indexOf("R:");

  if (l_index != -1 && r_index != -1) {
    int comma_index = cmd.indexOf(',');
    if (comma_index != -1) {
      String l_str = cmd.substring(l_index + 2, comma_index);
      String r_str = cmd.substring(r_index + 2);

      int left_pwm  = l_str.toInt();
      int right_pwm = r_str.toInt();

      setLeftMotors(left_pwm);
      setRightMotors(right_pwm);
    }
  }
}

// -----------------------------------------------------------------------------
// Send Telemetry (Encoders + IMU) to ROS 2
// -----------------------------------------------------------------------------
void sendTelemetry() {
  // Read atomic snapshot of encoder ticks
  noInterrupts();
  long l_ticks = left_encoder_ticks;
  long r_ticks = right_encoder_ticks;
  interrupts();

  // Output format: "E:<left_ticks>,<right_ticks>"
  Serial.print("E:");
  Serial.print(l_ticks);
  Serial.print(",");
  Serial.println(r_ticks);

  // Send IMU data if detected (MPU6050 or BNO055)
  if (active_imu == IMU_MPU6050) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Register 0x3B (ACCEL_XOUT_H)
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU6050_ADDR, 14, true) == 14) {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      int16_t temp = (Wire.read() << 8) | Wire.read(); (void)temp;
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      // Output format: "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>"
      Serial.print("I:");
      Serial.print(ax); Serial.print(",");
      Serial.print(ay); Serial.print(",");
      Serial.print(az); Serial.print(",");
      Serial.print(gx); Serial.print(",");
      Serial.print(gy); Serial.print(",");
      Serial.println(gz);
    }
  } else if (active_imu == IMU_BNO055) {
    // Read Linear Accel (0x08) and Gyro (0x14) from BNO055
    Wire.beginTransmission(BNO055_ADDR);
    Wire.write(0x08); // ACC_DATA_X_LSB
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(BNO055_ADDR, 6, true) == 6) {
      int16_t ax = Wire.read() | (Wire.read() << 8);
      int16_t ay = Wire.read() | (Wire.read() << 8);
      int16_t az = Wire.read() | (Wire.read() << 8);

      Wire.beginTransmission(BNO055_ADDR);
      Wire.write(0x14); // GYR_DATA_X_LSB
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom(BNO055_ADDR, 6, true) == 6) {
        int16_t gx = Wire.read() | (Wire.read() << 8);
        int16_t gy = Wire.read() | (Wire.read() << 8);
        int16_t gz = Wire.read() | (Wire.read() << 8);

        Serial.print("I:");
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.println(gz);
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Motor Control Functions
// -----------------------------------------------------------------------------
void setLeftMotors(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(pwm_val));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setRightMotors(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(pwm_val));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
```

#### Option B: Arduino Uno WITHOUT Encoders (`firmware/arduino_open_loop_controller.ino`)
*Use when using standard 4WD chassis kits without motor encoders:*
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/firmware/arduino_open_loop_controller.ino
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/firmware/arduino_open_loop_controller.ino
```
*Paste and save:*
```cpp
/**
 * ============================================================================
 * 4-Wheel Differential Drive Robot - Arduino Firmware (WITHOUT ENCODERS / OPEN-LOOP + IMU)
 * ============================================================================
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 * 
 * Workshop Edition for ROS 2 Mobile Robotics Workshop.
 * 
 * Use this sketch when your 4WD mobile robot does NOT have motor encoders
 * (e.g. standard yellow TT DC gearmotors without Hall sensors).
 * 
 * Features:
 *  1. Dual H-Bridge Motor Control (L298N).
 *  2. Open-Loop PWM control driven directly by /wheel_speed_commands from ROS 2.
 *  3. MPU6050 / BNO055 IMU integration via I2C (Pins A4-SDA, A5-SCL) with auto-detection.
 *  4. 1.0s Watchdog timer for fail-safe emergency motor stopping if ROS 2 disconnects.
 *  5. High-speed bidirectional Serial communication with ROS 2 @ 115200 baud.
 * 
 * Serial Protocol:
 *  - Incoming (from ROS 2): "L:<left_pwm>,R:<right_pwm>\n"   (-255 to 255)
 *  - Outgoing (to ROS 2):   "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n" (Raw IMU Feedback)
 * ============================================================================
 */

#include <Wire.h>

// -----------------------------------------------------------------------------
// 1. Motor Pin Definitions (L298N)
// -----------------------------------------------------------------------------
const int ENA = 5;    // Left Motors PWM Speed
const int IN1 = 7;    // Left Direction 1
const int IN2 = 8;    // Left Direction 2

const int ENB = 6;    // Right Motors PWM Speed
const int IN3 = 9;    // Right Direction 1
const int IN4 = 10;   // Right Direction 2

// -----------------------------------------------------------------------------
// 2. IMU Configuration (Supports both MPU6050 and BNO055)
// -----------------------------------------------------------------------------
const int MPU6050_ADDR = 0x68;
const int BNO055_ADDR  = 0x28;

enum ImuType { IMU_NONE, IMU_MPU6050, IMU_BNO055 };
ImuType active_imu = IMU_NONE;

// -----------------------------------------------------------------------------
// 3. Timing & Watchdog
// -----------------------------------------------------------------------------
unsigned long last_command_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 1000; // 1.0s fail-safe stop
unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 50 Hz telemetry

// -----------------------------------------------------------------------------
// 4. Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  // Motor Pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Stop Motors initially
  stopMotors();

  // Initialize I2C for IMU
  Wire.begin();
  delay(100);

  // Auto-detect MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    active_imu = IMU_MPU6050;
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission();
    Serial.println("STATUS:IMU MPU6050 DETECTED (0x68)");
  } else {
    // Check BNO055
    Wire.beginTransmission(BNO055_ADDR);
    if (Wire.endTransmission() == 0) {
      active_imu = IMU_BNO055;
      Serial.println("STATUS:IMU BNO055 DETECTED (0x28)");
    } else {
      active_imu = IMU_NONE;
      Serial.println("STATUS:NO IMU DETECTED");
    }
  }

  Serial.println("STATUS:ARDUINO OPEN-LOOP MOTOR CONTROLLER READY");
}

// -----------------------------------------------------------------------------
// 5. Main Loop
// -----------------------------------------------------------------------------
void loop() {
  // 1. Process incoming serial commands from ROS 2
  processSerialCommands();

  // 2. Watchdog: Emergency stop if no command received within 1 second
  if (millis() - last_command_time > WATCHDOG_TIMEOUT_MS) {
    stopMotors();
  }

  // 3. Periodic IMU Telemetry to ROS 2 (50 Hz)
  if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = millis();
    sendImuTelemetry();
  }
}

// -----------------------------------------------------------------------------
// 6. Motor Control Functions
// -----------------------------------------------------------------------------
void setLeftMotor(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwm_val);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setRightMotor(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -pwm_val);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// -----------------------------------------------------------------------------
// 7. Serial Communication with ROS 2
// -----------------------------------------------------------------------------
void processSerialCommands() {
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("L:") && cmd.indexOf(",R:") != -1) {
      int r_idx = cmd.indexOf(",R:");
      String left_str = cmd.substring(2, r_idx);
      String right_str = cmd.substring(r_idx + 3);

      int left_pwm = left_str.toInt();
      int right_pwm = right_str.toInt();

      setLeftMotor(left_pwm);
      setRightMotor(right_pwm);
      last_command_time = millis();
    }
  }
}

void sendImuTelemetry() {
  if (active_imu == IMU_MPU6050) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    if (Wire.available() >= 14) {
      int16_t ax = Wire.read() << 8 | Wire.read();
      int16_t ay = Wire.read() << 8 | Wire.read();
      int16_t az = Wire.read() << 8 | Wire.read();
      int16_t temp = Wire.read() << 8 | Wire.read();
      int16_t gx = Wire.read() << 8 | Wire.read();
      int16_t gy = Wire.read() << 8 | Wire.read();
      int16_t gz = Wire.read() << 8 | Wire.read();

      // Convert to SI units: m/s^2 and rad/s
      float ax_m_s2 = (float)ax / 16384.0 * 9.80665;
      float ay_m_s2 = (float)ay / 16384.0 * 9.80665;
      float az_m_s2 = (float)az / 16384.0 * 9.80665;
      float gx_rad_s = (float)gx / 131.0 * (PI / 180.0);
      float gy_rad_s = (float)gy / 131.0 * (PI / 180.0);
      float gz_rad_s = (float)gz / 131.0 * (PI / 180.0);

      Serial.print("I:");
      Serial.print(ax_m_s2, 3); Serial.print(",");
      Serial.print(ay_m_s2, 3); Serial.print(",");
      Serial.print(az_m_s2, 3); Serial.print(",");
      Serial.print(gx_rad_s, 3); Serial.print(",");
      Serial.print(gy_rad_s, 3); Serial.print(",");
      Serial.println(gz_rad_s, 3);
    }
  }
}
```

#### Option C: ESP32 with 4 Encoders + IMU (`firmware/esp32_motor_controller.ino`)
*Use when building high-performance 4WD robot with 4 independent encoders:*
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/firmware/esp32_motor_controller.ino
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/firmware/esp32_motor_controller.ino
```
*Paste and save:*
```cpp
/**
 * ============================================================================
 * 4-Wheel Differential Drive Robot - ESP32 Firmware (Encoders + IMU)
 * ============================================================================
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 * 
 * Workshop Edition for ROS 2 Mobile Robotics Workshop.
 * 
 * Hardware Compatibility:
 *  - ESP32 Development Board (30-pin or 38-pin ESP32-WROOM-32)
 *  - Motor Driver: L298N / TB6612FNG Dual H-Bridge
 *  - DC Gearmotors with Quadrature Encoders
 *  - MPU6050 6-Axis IMU (I2C)
 * 
 * Pinout Assignments (ESP32):
 *  - Left Motor:   ENA=GPIO 25 (PWM), IN1=GPIO 26, IN2=GPIO 27
 *  - Right Motor:  ENB=GPIO 14 (PWM), IN3=GPIO 12, IN4=GPIO 13
 *  - Left Encoder: ENCA=GPIO 18, ENCB=GPIO 19
 *  - Right Encoder:ENCA=GPIO 16, ENCB=GPIO 17
 *  - I2C IMU:      SDA=GPIO 21,  SCL=GPIO 22
 * ============================================================================
 */

#include <Wire.h>

// -----------------------------------------------------------------------------
// 1. Motor Pin Definitions
// -----------------------------------------------------------------------------
const int PIN_ENA = 25;   // Left Motor PWM
const int PIN_IN1 = 26;   // Left Direction 1
const int PIN_IN2 = 27;   // Left Direction 2

const int PIN_ENB = 14;   // Right Motor PWM
const int PIN_IN3 = 12;   // Right Direction 1
const int PIN_IN4 = 13;   // Right Direction 2

// ESP32 PWM (LEDC) Configuration
const int PWM_FREQ = 1000;       // 1 kHz PWM frequency
const int PWM_RES  = 8;          // 8-bit resolution (0-255)
const int LEDC_CH_LEFT  = 0;     // PWM Channel 0
const int LEDC_CH_RIGHT = 1;     // PWM Channel 1

// -----------------------------------------------------------------------------
// 2. Encoder Pin Definitions & Counters
// -----------------------------------------------------------------------------
const int PIN_ENC_LEFT_A  = 18;  // Interrupt pin
const int PIN_ENC_LEFT_B  = 19;
const int PIN_ENC_RIGHT_A = 16;  // Interrupt pin
const int PIN_ENC_RIGHT_B = 17;

volatile long left_encoder_ticks  = 0;
volatile long right_encoder_ticks = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// -----------------------------------------------------------------------------
// 3. IMU Configuration (Supports both MPU6050 and BNO055)
// -----------------------------------------------------------------------------
const int MPU6050_ADDR = 0x68;
const int BNO055_ADDR  = 0x28;

enum ImuType { IMU_NONE, IMU_MPU6050, IMU_BNO055 };
ImuType active_imu = IMU_NONE;

// -----------------------------------------------------------------------------
// 4. Timing & Safety Watchdog
// -----------------------------------------------------------------------------
const unsigned long TIMEOUT_MS    = 1000;  // Stop motors if no command for 1 second
const unsigned long TELEMETRY_MS  = 50;    // Telemetry rate 20 Hz
unsigned long last_cmd_time       = 0;
unsigned long last_telemetry_time = 0;

// -----------------------------------------------------------------------------
// Interrupt Service Routines (ISRs) for ESP32
// -----------------------------------------------------------------------------
void IRAM_ATTR isrLeftEncoder() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(PIN_ENC_LEFT_B) == HIGH) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isrRightEncoder() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(PIN_ENC_RIGHT_B) == HIGH) {
    right_encoder_ticks++;
  } else {
    right_encoder_ticks--;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  // Configure Motor Direction Pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Configure ESP32 LEDC PWM Channels
  ledcSetup(LEDC_CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_ENA, LEDC_CH_LEFT);

  ledcSetup(LEDC_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_ENB, LEDC_CH_RIGHT);

  stopMotors();

  // Configure Encoder Pins with Pullups
  pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isrRightEncoder, RISING);

  // Initialize I2C on standard ESP32 pins (SDA=21, SCL=22)
  Wire.begin(21, 22);
  
  // 1. Try MPU6050 (0x68)
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    active_imu = IMU_MPU6050;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
  } else {
    // 2. Try Bosch BNO055 (0x28)
    Wire.beginTransmission(BNO055_ADDR);
    if (Wire.endTransmission() == 0) {
      active_imu = IMU_BNO055;
      Wire.beginTransmission(BNO055_ADDR);
      Wire.write(0x3D); // OPR_MODE
      Wire.write(0x08); // IMU mode
      Wire.endTransmission();
    }
  }

  Serial.println("==================================================");
  Serial.println("✓ 4-Wheel Robot ESP32 Firmware Ready!");
  Serial.println("  LEDC PWM Channels 0 & 1 Initialized (1 kHz)");
  Serial.println("  Hardware Interrupt Encoders (GPIO 18, 19, 16, 17)");
  Serial.print("  IMU: ");
  if (active_imu == IMU_MPU6050) {
    Serial.println("MPU-6050 Detected & Initialized ✓");
  } else if (active_imu == IMU_BNO055) {
    Serial.println("Bosch BNO-055 Detected & Initialized ✓");
  } else {
    Serial.println("None Detected (Skipped)");
  }
  Serial.println("==================================================");

  last_cmd_time = millis();
  last_telemetry_time = millis();
}

// -----------------------------------------------------------------------------
// Main Loop
// -----------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // 1. Process incoming commands from ROS 2
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      parseAndExecuteCommand(input);
      last_cmd_time = now;
    }
  }

  // 2. Safety Watchdog
  if (now - last_cmd_time > TIMEOUT_MS) {
    stopMotors();
  }

  // 3. Send Telemetry to ROS 2 (20 Hz)
  if (now - last_telemetry_time >= TELEMETRY_MS) {
    last_telemetry_time = now;
    sendTelemetry();
  }
}

// -----------------------------------------------------------------------------
// Parse Serial Command: "L:<left>,R:<right>"
// -----------------------------------------------------------------------------
void parseAndExecuteCommand(String cmd) {
  int l_index = cmd.indexOf("L:");
  int r_index = cmd.indexOf("R:");

  if (l_index != -1 && r_index != -1) {
    int comma_index = cmd.indexOf(',');
    if (comma_index != -1) {
      String l_str = cmd.substring(l_index + 2, comma_index);
      String r_str = cmd.substring(r_index + 2);

      int left_pwm  = l_str.toInt();
      int right_pwm = r_str.toInt();

      setLeftMotors(left_pwm);
      setRightMotors(right_pwm);
    }
  }
}

// -----------------------------------------------------------------------------
// Send Telemetry (Encoders + IMU)
// -----------------------------------------------------------------------------
void sendTelemetry() {
  portENTER_CRITICAL(&mux);
  long l_ticks = left_encoder_ticks;
  long r_ticks = right_encoder_ticks;
  portEXIT_CRITICAL(&mux);

  // Send encoder ticks: "E:<left_ticks>,<right_ticks>"
  Serial.print("E:");
  Serial.print(l_ticks);
  Serial.print(",");
  Serial.println(r_ticks);

  // Send IMU data if detected (MPU6050 or BNO055)
  if (active_imu == IMU_MPU6050) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU6050_ADDR, 14, true) == 14) {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      int16_t temp = (Wire.read() << 8) | Wire.read(); (void)temp;
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      Serial.print("I:");
      Serial.print(ax); Serial.print(",");
      Serial.print(ay); Serial.print(",");
      Serial.print(az); Serial.print(",");
      Serial.print(gx); Serial.print(",");
      Serial.print(gy); Serial.print(",");
      Serial.println(gz);
    }
  } else if (active_imu == IMU_BNO055) {
    Wire.beginTransmission(BNO055_ADDR);
    Wire.write(0x08); // ACC_DATA_X_LSB
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(BNO055_ADDR, 6, true) == 6) {
      int16_t ax = Wire.read() | (Wire.read() << 8);
      int16_t ay = Wire.read() | (Wire.read() << 8);
      int16_t az = Wire.read() | (Wire.read() << 8);

      Wire.beginTransmission(BNO055_ADDR);
      Wire.write(0x14); // GYR_DATA_X_LSB
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom(BNO055_ADDR, 6, true) == 6) {
        int16_t gx = Wire.read() | (Wire.read() << 8);
        int16_t gy = Wire.read() | (Wire.read() << 8);
        int16_t gz = Wire.read() | (Wire.read() << 8);

        Serial.print("I:");
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.println(gz);
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Motor Control (ESP32 LEDC PWM)
// -----------------------------------------------------------------------------
void setLeftMotors(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(LEDC_CH_LEFT, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    ledcWrite(LEDC_CH_LEFT, abs(pwm_val));
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(LEDC_CH_LEFT, 0);
  }
}

void setRightMotors(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    ledcWrite(LEDC_CH_RIGHT, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    ledcWrite(LEDC_CH_RIGHT, abs(pwm_val));
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    ledcWrite(LEDC_CH_RIGHT, 0);
  }
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  ledcWrite(LEDC_CH_LEFT, 0);
  ledcWrite(LEDC_CH_RIGHT, 0);
}
```

#### Option D: ESP32 WITHOUT Encoders / Open-Loop (`firmware/esp32_open_loop_controller.ino`)
*Use when deploying on ESP32 without quadrature motor encoders (utilizes high-frequency LEDC hardware PWM):*
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/firmware/esp32_open_loop_controller.ino
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_firmware/firmware/esp32_open_loop_controller.ino
```
*Paste and save:*
```cpp
/**
 * ============================================================================
 * 4-Wheel Differential Drive Robot - ESP32 Firmware (WITHOUT ENCODERS / OPEN-LOOP + IMU)
 * ============================================================================
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 * 
 * Workshop Edition for ROS 2 Mobile Robotics Workshop.
 * 
 * Use this sketch when deploying on ESP32 without quadrature motor encoders
 * (e.g., standard TT DC gearmotors or open-loop skid-steer chassis).
 * 
 * Features:
 *  1. Dual H-Bridge Motor Control (L298N / TB6612FNG).
 *  2. High-precision hardware LEDC PWM (compatible with ESP32 Core 2.x & 3.x).
 *  3. Open-loop velocity control directly mapped from ROS 2 Twist / PWM commands.
 *  4. MPU6050 / BNO055 IMU integration on I2C (Pins GPIO 21-SDA, GPIO 22-SCL).
 *  5. 1.0s Watchdog timer for fail-safe automatic motor cutoff on serial loss.
 *  6. 115200 Baud high-speed bidirectional communication with serial_hardware_bridge.py.
 * 
 * Serial Protocol:
 *  - Incoming (from ROS 2): "L:<left_pwm>,R:<right_pwm>\n"   (-255 to 255)
 *  - Outgoing (to ROS 2):   "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n" (Raw IMU Feedback)
 * ============================================================================
 */

#include <Wire.h>

// -----------------------------------------------------------------------------
// 1. ESP32 Pin Definitions
// -----------------------------------------------------------------------------
// Left Motor Channel
const int PIN_ENA = 25;   // Left PWM Speed Pin
const int PIN_IN1 = 26;   // Left Direction 1
const int PIN_IN2 = 27;   // Left Direction 2

// Right Motor Channel
const int PIN_ENB = 14;   // Right PWM Speed Pin
const int PIN_IN3 = 12;   // Right Direction 1
const int PIN_IN4 = 13;   // Right Direction 2

// I2C IMU Pins
const int PIN_SDA = 21;
const int PIN_SCL = 22;

// LEDC Hardware PWM Configuration
const int PWM_FREQ = 20000;    // 20 kHz ultrasonic (silent motors)
const int PWM_RESOLUTION = 8;  // 8-bit resolution (0 - 255)
const int PWM_CH_LEFT = 0;
const int PWM_CH_RIGHT = 1;

// -----------------------------------------------------------------------------
// 2. IMU Configuration
// -----------------------------------------------------------------------------
const int MPU6050_ADDR = 0x68;
const int BNO055_ADDR  = 0x28;

enum ImuType { IMU_NONE, IMU_MPU6050, IMU_BNO055 };
ImuType active_imu = IMU_NONE;

// -----------------------------------------------------------------------------
// 3. Timing & Watchdog
// -----------------------------------------------------------------------------
unsigned long last_command_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 1000; // 1.0s fail-safe stop
unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 50 Hz telemetry

// -----------------------------------------------------------------------------
// 4. Helper: LEDC PWM write abstraction (Supports ESP32 Core 2.x & 3.x)
// -----------------------------------------------------------------------------
void initPwmPin(int pin, int channel) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(pin, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pin, channel);
#endif
}

void writePwm(int pin, int channel, int duty) {
  duty = constrain(duty, 0, 255);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// -----------------------------------------------------------------------------
// 5. Motor Control Functions
// -----------------------------------------------------------------------------
void setMotorSpeeds(int left_pwm, int right_pwm) {
  // Left Motor Direction
  if (left_pwm > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (left_pwm < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
  writePwm(PIN_ENA, PWM_CH_LEFT, abs(left_pwm));

  // Right Motor Direction
  if (right_pwm > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else if (right_pwm < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
  }
  writePwm(PIN_ENB, PWM_CH_RIGHT, abs(right_pwm));
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  writePwm(PIN_ENA, PWM_CH_LEFT, 0);
  writePwm(PIN_ENB, PWM_CH_RIGHT, 0);
}

// -----------------------------------------------------------------------------
// 6. Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  // Direction Pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // PWM Pins
  initPwmPin(PIN_ENA, PWM_CH_LEFT);
  initPwmPin(PIN_ENB, PWM_CH_RIGHT);

  stopMotors();

  // I2C for IMU
  Wire.begin(PIN_SDA, PIN_SCL);
  delay(100);

  // Detect MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    active_imu = IMU_MPU6050;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission();
    Serial.println("STATUS:ESP32_OPEN_LOOP_MPU6050_READY");
  } else {
    // Detect BNO055
    Wire.beginTransmission(BNO055_ADDR);
    if (Wire.endTransmission() == 0) {
      active_imu = IMU_BNO055;
      Serial.println("STATUS:ESP32_OPEN_LOOP_BNO055_READY");
    } else {
      active_imu = IMU_NONE;
      Serial.println("STATUS:ESP32_OPEN_LOOP_NO_IMU");
    }
  }

  last_command_time = millis();
}

// -----------------------------------------------------------------------------
// 7. Loop: Command Processing, Watchdog & Telemetry
// -----------------------------------------------------------------------------
void loop() {
  // 1. Process Serial Commands from ROS 2
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("L:") && line.indexOf(",R:") > 0) {
      int r_idx = line.indexOf(",R:");
      int left_pwm = line.substring(2, r_idx).toInt();
      int right_pwm = line.substring(r_idx + 3).toInt();

      left_pwm = constrain(left_pwm, -255, 255);
      right_pwm = constrain(right_pwm, -255, 255);

      setMotorSpeeds(left_pwm, right_pwm);
      last_command_time = millis();
    }
  }

  // 2. Failsafe Watchdog Check
  if (millis() - last_command_time > WATCHDOG_TIMEOUT_MS) {
    stopMotors();
  }

  // 3. Publish Telemetry at 50 Hz
  if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = millis();

    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;

    if (active_imu == IMU_MPU6050) {
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDR, 14, true);

      if (Wire.available() >= 14) {
        ax = (Wire.read() << 8) | Wire.read();
        ay = (Wire.read() << 8) | Wire.read();
        az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temp
        gx = (Wire.read() << 8) | Wire.read();
        gy = (Wire.read() << 8) | Wire.read();
        gz = (Wire.read() << 8) | Wire.read();
      }
    }

    Serial.print("I:");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }
}
```

---

### ⚡ MILESTONE 6 VERIFICATION (Hardware Bridge & Telemetry Execution)

> [!IMPORTANT]
> **⚠️ Beginner Rule: In EVERY new terminal tab you open, always run:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_mobile_robot_ws/install/setup.bash
> ```

#### Terminal 1: Launch Master Simulation Bringup (Gazebo + RViz TOGETHER!)
```bash
cd ~/ros2_mobile_robot_ws
colcon build --packages-select mobile_robot_firmware --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_bringup simulate_robot.launch.py
```

#### Terminal 2: Launch Serial Hardware Bridge (Choose Mode)

* **Option A: Physical Hardware Mode (Microcontroller connected via USB):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_firmware firmware.launch.py port:=/dev/ttyUSB0
```

* **Option B: Mock Simulation Mode (Without physical microcontroller plugged in):**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_firmware firmware.launch.py
```

#### Terminal 3: Echo Real-Time Encoders & IMU Telemetry

* **Test 1: Echo encoder counts published by microcontroller / bridge:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /wheel_encoder_ticks --once
```

* **Test 2: Echo IMU linear acceleration & angular velocity:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /imu/data_raw --once
```

* **Test 3: Send motor speed commands to firmware:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -1 /wheel_speed_commands std_msgs/msg/Float32MultiArray "data: [5.0, 5.0]"
```

---

<a id="milestone-7" name="milestone-7"></a>
## Milestone 7
### Real Robot Hardware Bringup on Raspberry Pi (`mobile_robot_bringup` - `hardware_robot.launch.py`)

In Milestone 7, you deploy the entire robot intelligence stack onto a physical onboard Single Board Computer (Raspberry Pi 4 / 5 or Jetson Nano).

---

<a id="raspberry-pi-setup" name="raspberry-pi-setup"></a>
## Raspberry Pi Setup
### 🍓 Raspberry Pi 4 / 5 Onboard Computer Hardware & OS Setup

To prepare your Raspberry Pi as the autonomous robot's brain, follow these standard hardware and system configuration steps:

#### 1. Operating System & ROS 2 Installation
* **OS:** Ubuntu Server 22.04 LTS (64-bit ARM64) flashed via Raspberry Pi Imager.
* **ROS 2:** ROS 2 Humble Hawksbill (`ros-humble-ros-base` or `ros-humble-desktop`).

#### 2. Hardware UART & Serial Bus Enablement (`/boot/firmware/config.txt` or `/boot/config.txt`)
To enable hardware serial, I2C, and camera interfaces, edit the boot configuration:
```bash
sudo nano /boot/firmware/config.txt
# (On older Ubuntu releases: sudo nano /boot/config.txt)
```
Add the following configuration flags at the bottom of the file:
```ini
# Enable Hardware UART, I2C, and High-Speed SPI
enable_uart=1
dtparam=i2c_arm=on
dtparam=spi=on

# Increase USB Bus Current Output for LiDAR and Microcontrollers (Pi 4/5)
max_usb_current=1

# Enable Camera Interface
camera_auto_detect=1
start_x=1
gpu_mem=128
```
Save with `Ctrl+O`, `Enter`, and exit with `Ctrl+X`. Then disable Linux serial console login to free hardware UART:
```bash
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
```

#### 3. Persistent USB Device Symlinks (`udev` Rules)
When multiple USB serial devices (Arduino, ESP32, RPLiDAR, USB Camera) are connected to the Raspberry Pi, Linux may swap `/dev/ttyUSB0` and `/dev/ttyUSB1` upon reboot or re-plugging. Create persistent named symlinks:
```bash
sudo nano /etc/udev/rules.d/99-robot-hardware.rules
```
Paste the following vendor rules:
```udev
# Arduino Uno / Nano (CH340 or FTDI or CDC-ACM) -> /dev/robot_mcu
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="robot_mcu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", MODE="0666", SYMLINK+="robot_mcu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="robot_mcu"

# RPLiDAR A1 / A2 / C1 -> /dev/rplidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE="0666", SYMLINK+="rplidar"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", MODE="0666", SYMLINK+="rplidar"
```
Reload and trigger the udev daemon:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 4. Serial Port Low-Latency Optimization (1ms Latency Timer)
By default, Linux buffers FTDI USB serial packets for 16ms before flushing to user space. To achieve high-frequency 50Hz odometry and 10Hz LiDAR scanning with zero lag:
```bash
# Add low latency rule for all USB serial interfaces
sudo nano /etc/udev/rules.d/99-usb-low-latency.rules
```
Paste:
```udev
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="cp210x", ATTR{latency_timer}="1"
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ch341", ATTR{latency_timer}="1"
```
Reload rules:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

### Step 7.1: Create `launch/hardware_robot.launch.py`
```bash
touch ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/launch/hardware_robot.launch.py
gedit ~/ros2_mobile_robot_ws/src/mobile_robot_bringup/launch/hardware_robot.launch.py
```
*Paste and save:*
```python
#!/usr/bin/env python3
"""
================================================================================
Hardware Robot Bringup Launch File (Raspberry Pi / Physical Robot)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Launches the complete hardware stack for the real 4-wheel mobile robot:
 1. Robot State Publisher (URDF/Xacro with 4 wheels, LiDAR, Camera, IMU)
 2. Differential Drive Kinematics Controller (C++ or Python)
 3. Serial Hardware Bridge (Communicates with Arduino / ESP32 over USB Serial,
    reads encoder ticks and IMU data, sends motor PWM commands)
 4. Twist Mux (Prioritized velocity multiplexer with emergency /e_stop lock)
 5. Optional RViz2 (Default: false, for headless Raspberry Pi operation)
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('mobile_robot_description')
    ctrl_pkg = get_package_share_directory('mobile_robot_controller')
    bringup_pkg = get_package_share_directory('mobile_robot_bringup')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'mobile_robot.urdf.xacro')
    rviz_file = os.path.join(desc_pkg, 'rviz', 'simulation.rviz')
    controller_params = os.path.join(ctrl_pkg, 'config', 'controller_params.yaml')

    # Launch Arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Microcontroller serial port (e.g. /dev/ttyUSB0 or /dev/ttyACM0)'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial communication baud rate'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Set to true for C++ controller, false for Python controller'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz2 (set false on headless Raspberry Pi, true on laptop)'
    )

    use_twist_mux_arg = DeclareLaunchArgument(
        'use_twist_mux',
        default_value='true',
        description='Enable Twist Mux velocity multiplexer'
    )

    # Robot Description
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2a. C++ Controller Node
    controller_cpp_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller_cpp',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            controller_params,
            {'use_sim_time': False, 'publish_joint_states': True}
        ],
        condition=IfCondition(LaunchConfiguration('use_cpp'))
    )

    # 2b. Python Controller Node
    controller_py_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller_py',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            controller_params,
            {'use_sim_time': False, 'publish_joint_states': True}
        ],
        condition=UnlessCondition(LaunchConfiguration('use_cpp'))
    )

    # 3. Serial Hardware Bridge (Talks to Arduino / ESP32)
    serial_bridge_node = Node(
        package='mobile_robot_firmware',
        executable='serial_hardware_bridge.py',
        name='serial_hardware_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }]
    )

    # 4. Twist Mux Velocity Multiplexer & Joystick Teleop
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'joystick_teleop.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'cmd_vel_out': 'cmd_vel_raw'}.items(),
        condition=IfCondition(LaunchConfiguration('use_twist_mux'))
    )

    # 5. Safety Zone Controller (Red Zone stop, Yellow Zone 2x slowdown, auto-resume, RViz toggle button)
    safety_controller_node = Node(
        package='mobile_robot_bringup',
        executable='safety_zone_controller.py',
        name='safety_zone_controller',
        output='screen'
    )

    # 6. Optional RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        use_cpp_arg,
        launch_rviz_arg,
        use_twist_mux_arg,
        robot_state_publisher_node,
        controller_cpp_node,
        controller_py_node,
        serial_bridge_node,
        twist_mux_launch,
        safety_controller_node,
        rviz_node
    ])
```

---

### Step 7.2: Deploying Code to Raspberry Pi (PC to Robot Workflow)

Here is the exact step-by-step workflow to send your code from your development PC to the Raspberry Pi over Wi-Fi, SSH into the robot, and run the physical hardware stack:

#### 1. Find the Raspberry Pi IP Address
Make sure your development laptop and Raspberry Pi are connected to the same Wi-Fi network:
* On the Raspberry Pi:
  ```bash
  hostname -I
  ```
* Or scan from your PC:
  ```bash
  ping raspberrypi.local
  # Note the IP address, e.g. 192.168.1.150
  ```

#### 2. Send Code from PC to Raspberry Pi using `scp` (or `rsync`)

* **Step 2.1: Create workspace directory on the Raspberry Pi:**
```bash
ssh ubuntu@192.168.1.150 "mkdir -p ~/ros2_mobile_robot_ws/src"
```

* **Step 2.2: Copy the entire `src` directory from PC to Raspberry Pi:**
```bash
scp -r ~/ros2_mobile_robot_ws/src ubuntu@192.168.1.150:~/ros2_mobile_robot_ws/
```

> [!TIP]
> **Fast Incremental Sync with `rsync`:**  
> For daily development, `rsync` transfers only modified files and automatically skips build/log folders:
> ```bash
> rsync -avz --delete --exclude 'build' --exclude 'install' --exclude 'log' ~/ros2_mobile_robot_ws/ ubuntu@192.168.1.150:~/ros2_mobile_robot_ws/
> ```

#### 3. Connect to the Robot via SSH
```bash
ssh ubuntu@192.168.1.150
```

#### 4. Build the Workspace on the Raspberry Pi
Inside the Raspberry Pi SSH session:

* **Step 4.1: Install dependencies and build workspace:**
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

* **Step 4.2: Ensure serial permissions for USB microcontroller and LiDAR ports:**
```bash
sudo usermod -a -G dialout $USER
```
*(Log out and re-SSH into the Raspberry Pi if this is your first time setting up permissions).*

#### 5. Configure Network Discovery (`ROS_DOMAIN_ID`)
To enable seamless ROS 2 DDS communication between the Raspberry Pi and your remote PC over Wi-Fi, assign both machines the **same `ROS_DOMAIN_ID`**:

* **Step 5.1: Set ROS_DOMAIN_ID on Raspberry Pi:**
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

* **Step 5.2: Set ROS_DOMAIN_ID on Remote PC:**
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

---

### ⚡ MILESTONE 7 VERIFICATION (Running the Physical Robot)

> [!IMPORTANT]
> **⚠️ Beginner Rule: In EVERY new terminal tab you open (on PC or Raspberry Pi), always run:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_mobile_robot_ws/install/setup.bash
> ```

#### Step 1: On the Raspberry Pi (via SSH)
Launch the complete hardware stack (robot description, differential drive controller, serial bridge to microcontroller, twist mux, and safety zone controller):
```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mobile_robot_bringup hardware_robot.launch.py port:=/dev/ttyUSB0
```

#### Step 2: Launch Sensors on Raspberry Pi (Separate SSH Terminals)

* **Option A: Launch RPLiDAR (A1 / A2 / C1):**
```bash
ros2 launch rplidar_ros rplidar_a1.launch.py serial_port:=/dev/ttyUSB1 frame_id:=lidar_link
```

* **Option B: Launch YDLidar (X4 / G4 / T-mini):**
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

* **Option C: Launch USB / CSI Camera:**
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

#### Step 3: On Remote PC (Teleoperation & Visualization)
Because both machines share `ROS_DOMAIN_ID=42`, all topics, sensor scans, and transforms stream across Wi-Fi automatically:
* **Terminal 1 (PC): Launch RViz to View Live Robot:**
  ```bash
  cd ~/ros2_mobile_robot_ws
  source install/setup.bash
  ros2 launch mobile_robot_description display.launch.py
  ```
* **Terminal 2 (PC): Plug Gamepad into PC and Drive:**
  ```bash
  cd ~/ros2_mobile_robot_ws
  source install/setup.bash
  ros2 launch mobile_robot_controller joystick_teleop.launch.py
  ```
  *Hold LB (Button 4)* and drive with the Left Stick while steering with the Right Stick!

---

<a id="troubleshooting-hardware" name="troubleshooting-hardware"></a>
## Troubleshooting Hardware
### 🔌 Track 2: Embedded Hardware & Serial Troubleshooting Guide

Deploying from simulation to physical hardware (Raspberry Pi 4/5, Arduino Uno, ESP32, L298N/TB6612FNG motor drivers, and quadrature encoders) introduces real-world electrical, serial, and mechanical challenges. Use this diagnosis matrix and detailed guide to resolve embedded hardware bugs:

### 📋 Embedded Hardware & Serial Diagnosis Matrix

| Issue / Error Symptom | Root Cause | Diagnosis & Immediate Resolution |
| :--- | :--- | :--- |
| **`[Errno 13] Permission denied: '/dev/ttyUSB0'`** | Current Linux user is not a member of the serial `dialout` group. | Run: `sudo usermod -a -G dialout $USER` and log out then log back in. Temporarily override: `sudo chmod 666 /dev/ttyUSB0`. |
| **`Serial Bridge Drops to Mock Mode Automatically`** | Device plugged into `/dev/ttyACM0` (Arduino Uno CDC) instead of `/dev/ttyUSB0` (FTDI/CH340). | Check connected devices with `ls -l /dev/tty*` and launch with: `ros2 launch mobile_robot_firmware firmware.launch.py port:=/dev/ttyACM0`. |
| **`Robot Moves Backward on Positive /cmd_vel`** | DC motor polarity inverted or left/right motor wiring swapped. | Invert PWM sign in the `.ino` firmware sketch or physically swap the positive and negative leads on the motor driver output screw terminals. |
| **`Encoder Tick Jitter / Huge Position Drift`** | Inductive motor EMI ground noise interfering with hardware interrupt pins. | Add a $0.1\mu\text{F}$ decoupling ceramic capacitor across the DC motor power terminals and ensure logic GND and motor power GND are connected at a single star point. |
| **`IMU 0x68 / 0x69 Freezes on I2C Bus`** | I2C line bus lockup or missing pull-up resistors on SDA/SCL lines. | Add $4.7\text{k}\Omega$ pull-up resistors between SDA/SCL and 3.3V, and ensure `Wire.setWireTimeout(3000, true)` is present in firmware initialization. |
| **`ESP32 Stuck in Bootloader Loop`** | Encoders or motor inputs connected to strapping pins (GPIO 0, 2, 12, or 15). | Move encoder inputs to safe interrupt-capable GPIO pins: GPIO 18, 19, 21, 22, 23, 25, 26. |
| **`UART Telemetry Packet Drop at 115200 Baud`** | Blocking `delay()` calls in the microcontroller main loop causing buffer overflow. | Use non-blocking `millis()` loop timer at 50 Hz (`if (millis() - last_time >= 20)`) as implemented in the provided firmware sketches. |
| **`USB Disconnection / Cable Re-plug Crash`** | Linux assigns a new port index (e.g. `/dev/ttyUSB1`) upon re-plugging. | Create a persistent udev rule: `SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="robot_mcu"`, then launch with `port:=/dev/robot_mcu`. |
| **`Spontaneous MCU Reboots (Brownouts)`** | Powering MCU and motors from the same 5V rail; motor acceleration causes voltage drop. | Implement strict power isolation: 12V LiPo directly to motor driver $V_{in}$, clean 5V power bank/buck converter to Raspberry Pi/MCU. Common GND tied together. |
| **`2D LiDAR Connection Timeouts & Lag`** | Linux serial latency timer defaults to 16ms buffering. | Run: `echo 1 \| sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` to force 1ms ultra-low latency streaming. |
| **`QoS Reliability Mismatch (Silent Drop)`** | RPLiDAR publishes `SensorDataQoS` (Best Effort) while subscriber uses default Reliable. | Subscribe using `rclcpp::SensorDataQoS()` (C++) or `qos_profile_sensor_data` (Python) to prevent packet dropping. |
| **`Emergency Stop (/safety_stop) Lock Latching`** | Twist multiplexer lock remains active after an obstacle is removed. | Manually unlock twist mux with `ros2 topic pub -1 /safety_stop std_msgs/msg/Bool "data: false"`. |

---

### Detailed Diagnosis & Step-by-Step Fixes

#### 1. Serial Port Permission Denied (`/dev/ttyUSB0` or `/dev/ttyACM0`)
* **Error Message:** `PermissionError: [Errno 13] Permission denied: '/dev/ttyUSB0'`
* **Root Cause:** On Linux, serial UART devices belong to the `dialout` group. By default, regular user accounts do not have read/write access.
* **Resolution:**
  Add your Linux user to the `dialout` group:
  ```bash
  sudo usermod -a -G dialout $USER
  ```
  Log out and log back in, or run `newgrp dialout` to refresh permissions.
  You can also create persistent udev rules:
  ```bash
  echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-usb-serial.rules
  echo 'KERNEL=="ttyACM*", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-usb-serial.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

#### 2. Serial Communication Baud Rate Mismatch & Corrupted Packets
* **Symptom:** Bridge prints `Malformed packet` or gibberish characters (e.g. `⸮⸮⸮`).
* **Root Cause:** Mismatch between the microcontroller firmware baud rate (`Serial.begin(115200);`) and the ROS 2 serial hardware bridge launch argument.
* **Resolution:**
  Both ends must be strictly set to **115200 baud**.
  Ensure every packet sent from the MCU ends with a newline character (`\n` via `Serial.println()`).
  Test the raw serial feed directly with:
  ```bash
  screen /dev/ttyUSB0 115200
  # Press Ctrl+A then K to quit screen
  ```

#### 3. Motors Spinning in Reverse or Robot Turning Backwards
* **Symptom:** The robot drives backward when commanded forward, or turns left when commanded right.
* **Root Cause 1 - Motor Wiring Polarity:** DC motor terminals (OUT1/OUT2 on motor driver) are inverted relative to firmware PWM pin logic.
  * **Fix:** Swap the physical motor lead wires on the motor driver screw terminal for the affected side, or invert the sign multiplier in the firmware motor driver function.
* **Root Cause 2 - Encoder Phase Inversion:** Quadrature encoders have two channels (Phase A and Phase B). If A and B are swapped, the encoder counts negative ticks while the wheel turns forward, corrupting odometry dead-reckoning.
  * **Fix:** Swap the interrupt pins for Phase A and Phase B in the firmware sketch, or swap the physical signal wires.

#### 4. Severe Odometry Drift on Straight Lines
* **Symptom:** The robot drives straight for 2 meters, but RViz displays the robot turning or drifting by 30 degrees.
* **Root Cause:** Differences in effective wheel diameter, tire inflation/compression, or skid-steer wheel scrubbing:
  * **Fix 1 - Wheel Diameter Calibration:** Measure the actual rolling distance over 10 wheel revolutions: $D = \frac{\text{Distance}}{10 \cdot \pi}$. Update `wheel_radius` in `config/controller_params.yaml` or `config/ros2_robot_controller_params.yaml`.
  * **Fix 2 - Wheel Separation Calibration:** In 4-wheel differential drive, the effective kinematic track width ($L$) is typically 10% to 20% wider than the physical tape-measure distance due to wheel scrub. Adjust `wheel_separation_multiplier` (e.g. `1.12`) until a 360-degree physical rotation matches a 360-degree rotation in RViz.
  * **Fix 3 - Encoder CPR Verification:** Rotate the wheel exactly 1 full revolution by hand. Echo `/wheel_encoder_ticks`:
    ```bash
    ros2 topic echo /wheel_encoder_ticks --once
    ```
    Verify the tick count matches your motor's rated CPR (e.g., 330 ticks).

#### 5. Spontaneous Microcontroller Reboots (Brownouts Under Motor Acceleration)
* **Symptom:** Microcontroller restarts, serial bridge disconnects with `SerialException: device disconnected`, and motors suddenly stutter.
* **Root Cause:** Powering the Arduino or ESP32 from the same 5V rail as high-current DC motors. When DC motors accelerate, they draw peak stall currents (up to 2-3 Amperes), causing the voltage to drop below 4.5V and triggering the MCU's Brownout Detection (BOD) reset circuit.
* **Resolution:**
  * **STRICT POWER ISOLATION:** Power DC motors directly from a dedicated 12V Li-ion/LiFePO4 battery pack through the motor driver's $V_{in}$ terminal.
  * Power the Raspberry Pi and Arduino/ESP32 via a high-efficiency DC-DC buck converter (5V, 3A-5A rated) or separate power bank.
  * **COMMON GROUND:** Ensure the ground wire (GND) of the battery, motor driver, microcontroller, and Raspberry Pi are all connected together. Without a shared common ground reference, PWM signals float and motor control fails.

#### 6. 2D LiDAR Connection Timeouts & High Latency (RPLiDAR / YDLidar)
* **Symptom:** `Cannot open serial port`, `/scan` topic publishes at < 1 Hz, or `/scan` data is silently dropped by safety nodes.
* **Resolution:**
  1. Check device permissions: `sudo chmod 666 /dev/ttyUSB0` or ensure user is in `dialout` group (`sudo usermod -a -G dialout $USER`).
  2. For FTDI USB-to-UART converters, reduce the Linux serial latency timer from 16ms to 1ms:
     ```bash
     echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
     ```
  3. Verify LiDAR baud rate: RPLiDAR A1 runs at `115200`, RPLiDAR A2 runs at `115200` or `256000`, and YDLidar X4 runs at `128000`. Set the matching baud rate in the LiDAR launch file.
  4. **QoS Reliability Mismatch (Silent Packet Dropping):** If `ros2 topic hz /scan` shows the LiDAR publishing at 10 Hz, but your safety zone controller receives no messages, check subscriber QoS. Physical LiDAR drivers publish `/scan` using `SensorDataQoS` (Best Effort). Default ROS 2 subscribers use `Reliable` QoS and will silently drop all packets. Always subscribe with `rclcpp::SensorDataQoS()` (C++) or `qos_profile_sensor_data` (Python).

#### 7. Emergency Stop (`/safety_stop`) Lock Latching
* **Symptom:** Teleoperation or navigation commands produce zero wheel motion because the twist multiplexer is locked.
* **Verification:**
  Check twist mux lock states:
  ```bash
  ros2 topic echo /twist_mux/locks --once
  ```
  If `safety_stop` is `true`, unlock it manually:
  ```bash
  ros2 topic pub -1 /safety_stop std_msgs/msg/Bool "data: false"
  ```

---

### 📋 Pre-Flight Hardware Deployment Checklist

Before setting the robot on the floor, complete this 6-point verification:
1. [ ] **Wheels elevated:** Place the robot on a stand so all 4 wheels spin freely in the air.
2. [ ] **Battery voltage:** Verify motor battery is > 11.1V (for 3S LiPo) to prevent deep discharge.
3. [ ] **Common ground:** Confirm all GND lines are tied together.
4. [ ] **Serial bridge:** Run `ros2 run mobile_robot_firmware serial_hardware_bridge.py` and verify bidirectional telemetry (`/wheel_encoder_ticks` and `/wheel_speed_commands`).
5. [ ] **Spin test:** Publish `/cmd_vel` forward (+0.2 m/s). Verify both left and right wheels rotate forward.
6. [ ] **Failsafe test:** Depress LB on gamepad, deflect stick, then release LB. Verify wheels immediately coast to a complete stop.

---

### 🏁 Summary of Completed Milestones

| Milestone | Package | Key Deliverable | Purpose |
| :--- | :--- | :--- | :--- |
| [**Milestone 1**](#milestone-1) | `mobile_robot_description` | URDF, Xacro, Gazebo, RViz (`display.launch.py`) | Complete 3D mechanical robot model with physics & sensors |
| [**Milestone 2**](#milestone-2) | `mobile_robot_controller` | 3 Options: Dual Stack, Pure Coding (C++/Python), Pure `ros2_control` | Complete kinematics math and industrial control manager |
| [**Milestone 3**](#milestone-3) | `mobile_robot_controller` | Gamepad Teleop, Twist Mux & Relay | Dual stick steering with deadman safety and priority switching |
| [**Milestone 4**](#milestone-4) | `mobile_robot_bringup` | Safety Zone Controller (Both C++ & Python) | Active LiDAR emergency stopping & obstacle slowdown |
| [**Milestone 5**](#milestone-5) | `mobile_robot_bringup` | `simulate_robot.launch.py` | Master Simulation Bringup with Staged Timers |
| [**Milestone 6**](#milestone-6) | `mobile_robot_firmware` | Arduino (with/without enc), ESP32 (with/without enc), Bridge | Complete microcontroller firmware & serial telemetry bridge |
| [**Milestone 7**](#milestone-7) | `mobile_robot_bringup` | `hardware_robot.launch.py` & SCP Deploy | Full physical deployment on Raspberry Pi onboard robot |

---

### ⚡ Master All-in-One Workspace Comprehensive Test Suite

To verify all architecture tiers (Milestones 1–7, Track 2 Hardware Microcontrollers, and the 18 Multi-Simulator combinations) in one automated test pass:

```bash
cd ~/ros2_mobile_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Option 1: Run complete verification (Milestones 1-7 + Track 2 Hardware + Simulation Matrix)
python3 scripts/verify_all_lessons.py

# Option 2: Run core Milestones 1 to 7 unit tests only (~15s)
python3 scripts/test_mobile_robot_ws.py --milestones

# Option 3: Run Track 2 Hardware & Microcontroller two-way serial tests only (~10s)
python3 scripts/test_mobile_robot_ws.py --hardware

# Option 4: Run 18-combination Multi-Simulator & Multi-Controller Matrix only
python3 scripts/test_mobile_robot_ws.py --sim-matrix
```

*Expected Output for `python3 scripts/verify_all_lessons.py`:*
```text
======================================================================
 📋 FINAL COMPREHENSIVE TEST REPORT
======================================================================
[✅ PASS] Milestone 1: 3D URDF & Robot Description
[✅ PASS] Milestone 2: Differential Drive Controller (C++)
[✅ PASS] Milestone 2: Differential Drive Controller (Python)
[✅ PASS] Milestone 3: Teleoperation, Twist Mux & Twist Relay
[✅ PASS] Milestone 4: Safety Zone Controller (C++)
[✅ PASS] Milestone 4: Safety Zone Controller (Python)
[✅ PASS] Milestone 5: Master Simulation Bringup Launch Architecture
[✅ PASS] Milestone 6: Microcontroller Firmware & Serial Bridge (Mock Mode)
[✅ PASS] Milestone 7: Real Robot Hardware Bringup Launch
[✅ PASS] Arduino Uno (With Encoders + IMU)
[✅ PASS] Arduino Uno (Without Encoders / Open-Loop + IMU)
[✅ PASS] ESP32 (With Encoders + IMU)
[✅ PASS] ESP32 (Without Encoders / Open-Loop + IMU)
[✅ PASS] Serial Bridge Mock Fallback Mode
[✅ PASS] CLASSIC | ros2_control | CPP  Safety
[✅ PASS] CLASSIC | ros2_control | PY   Safety
[✅ PASS] CLASSIC | cpp          | CPP  Safety
[✅ PASS] CLASSIC | cpp          | PY   Safety
[✅ PASS] CLASSIC | py           | CPP  Safety
[✅ PASS] CLASSIC | py           | PY   Safety
[✅ PASS] IGN     | ros2_control | CPP  Safety
[✅ PASS] IGN     | ros2_control | PY   Safety
[✅ PASS] IGN     | cpp          | CPP  Safety
[✅ PASS] IGN     | cpp          | PY   Safety
[✅ PASS] IGN     | py           | CPP  Safety
[✅ PASS] IGN     | py           | PY   Safety
[✅ PASS] GZ      | ros2_control | CPP  Safety
[✅ PASS] GZ      | ros2_control | PY   Safety
[✅ PASS] GZ      | cpp          | CPP  Safety
[✅ PASS] GZ      | cpp          | PY   Safety
[✅ PASS] GZ      | py           | CPP  Safety
[✅ PASS] GZ      | py           | PY   Safety
======================================================================
🎉 100% SUCCESS: ALL 32/32 TESTS PASSED CLEANLY!
======================================================================
```

---

## 👤 Author & Maintainer

**Rahul Ramasamy**

* 🐙 **GitHub:** [https://github.com/rahul-r-joshua](https://github.com/rahul-r-joshua)
* 🌐 **Portfolio:** [https://rahul-r-joshua.github.io/portfolio/](https://rahul-r-joshua.github.io/portfolio/)
* 💼 **LinkedIn:** [https://www.linkedin.com/in/rahul-ramasamy-in/](https://www.linkedin.com/in/rahul-ramasamy-in/)
* 📧 **Email:** rahul.r.joshua123@gmail.com

---

### 💬 Support & Contact
For any bugs, issues, questions, or improvements, please feel free to open an issue or contact me directly via email or LinkedIn!

---

<div align="center">
  <sub>ROS 2 Mobile Robot Masterclass • Designed & Engineered for Autonomous Robotics Excellence.</sub>
</div>
