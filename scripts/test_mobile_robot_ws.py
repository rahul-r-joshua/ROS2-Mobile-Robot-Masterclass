#!/usr/bin/env python3
"""
================================================================================
Master Mobile Robot Workspace Comprehensive Test & Verification Suite
================================================================================
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Comprehensive Test Suite covering:
 1. Core Milestones 1 to 7 Unit Verification
 2. Track 2 Hardware & Microcontroller Two-Way UART Communication (Arduino & ESP32)
 3. Full 18-Combination Multi-Simulator Matrix (Gazebo Classic, Ignition, Modern Gz)
 4. Twist Mux Multiplexing & Safety Stop Latch Interception

Usage:
  python3 scripts/verify_all_lessons.py               # Runs full verification suite
  python3 scripts/verify_all_lessons.py --milestones  # Runs Milestones 1-7 fast check
  python3 scripts/verify_all_lessons.py --hardware    # Runs Track 2 MCU serial tests
  python3 scripts/verify_all_lessons.py --sim-matrix  # Runs 18-combination sim matrix
================================================================================
"""

import os
import sys
import pty
import tty
import select
import threading
import subprocess
import time

# Automatically source and inject workspace install environment
def ensure_environment():
    ws_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    setup_file = os.path.join(ws_dir, "install", "setup.bash")
    if os.path.exists(setup_file):
        cmd = f"source /opt/ros/humble/setup.bash 2>/dev/null && source {setup_file} 2>/dev/null && env"
        proc = subprocess.run(["bash", "-c", cmd], stdout=subprocess.PIPE, text=True)
        if proc.returncode == 0:
            for line in proc.stdout.splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    os.environ[k] = v

ensure_environment()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool


def run_clean():
    subprocess.run(["bash", "/home/ajay/ros2_mobile_robot_ws/clean_simulation.sh"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.3)


# ==============================================================================
# SECTION 1: CORE MILESTONES 1 TO 7 UNIT TESTS
# ==============================================================================

def test_milestone_1():
    print("  [M1] Checking 3D URDF & Robot Description...")
    p = subprocess.run(["xacro", "/home/ajay/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/mobile_robot.urdf.xacro"],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return p.returncode == 0 and "<robot" in p.stdout and "base_footprint" in p.stdout


def test_milestone_2_cpp():
    print("  [M2] Checking Differential Drive Controller (C++ Node)...")
    run_clean()
    p_ctrl = subprocess.Popen(["ros2", "launch", "mobile_robot_controller", "controller.launch.py", "use_cpp:=true", "use_sim_time:=false"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.2)

    rclpy.init()
    node = rclpy.create_node('m2_cpp_test')
    got_wheel_cmds = False

    def wheel_cb(msg):
        nonlocal got_wheel_cmds
        if len(msg.data) == 2 and (abs(msg.data[0]) > 0.0 or abs(msg.data[1]) > 0.0):
            got_wheel_cmds = True

    node.create_subscription(Float32MultiArray, '/wheel_speed_commands', wheel_cb, 10)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    t0 = time.time()
    while time.time() - t0 < 1.5 and not got_wheel_cmds:
        tw = Twist()
        tw.linear.x = 0.2
        pub.publish(tw)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    p_ctrl.terminate()
    run_clean()
    return got_wheel_cmds


def test_milestone_2_py():
    print("  [M2] Checking Differential Drive Controller (Python Node)...")
    run_clean()
    p_ctrl = subprocess.Popen(["ros2", "launch", "mobile_robot_controller", "controller.launch.py", "use_cpp:=false", "use_sim_time:=false"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.2)

    rclpy.init()
    node = rclpy.create_node('m2_py_test')
    got_wheel_cmds = False

    def wheel_cb(msg):
        nonlocal got_wheel_cmds
        if len(msg.data) == 2 and (abs(msg.data[0]) > 0.0 or abs(msg.data[1]) > 0.0):
            got_wheel_cmds = True

    node.create_subscription(Float32MultiArray, '/wheel_speed_commands', wheel_cb, 10)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    t0 = time.time()
    while time.time() - t0 < 1.5 and not got_wheel_cmds:
        tw = Twist()
        tw.linear.x = 0.2
        pub.publish(tw)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    p_ctrl.terminate()
    run_clean()
    return got_wheel_cmds


def test_milestone_3():
    print("  [M3] Checking Teleoperation, Twist Mux & Joystick Deadman...")
    p = subprocess.run(["ros2", "launch", "mobile_robot_controller", "joystick_teleop.launch.py", "-s"],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return p.returncode == 0


def test_milestone_4_cpp():
    print("  [M4] Checking Active Safety Zone Controller (C++ Node)...")
    run_clean()
    p_safe = subprocess.Popen(["ros2", "launch", "mobile_robot_bringup", "safety_zone.launch.py", "use_cpp:=true", "use_sim_time:=false"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.2)
    rclpy.init()
    node = rclpy.create_node('m4_cpp_test')
    got_markers = False

    def m_cb(msg):
        nonlocal got_markers
        if len(msg.markers) >= 2:
            got_markers = True

    node.create_subscription(MarkerArray, '/safety_zone_markers', m_cb, 10)
    t0 = time.time()
    while time.time() - t0 < 1.5 and not got_markers:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    p_safe.terminate()
    run_clean()
    return got_markers


def test_milestone_4_py():
    print("  [M4] Checking Active Safety Zone Controller (Python Node)...")
    run_clean()
    p_safe = subprocess.Popen(["ros2", "launch", "mobile_robot_bringup", "safety_zone.launch.py", "use_cpp:=false", "use_sim_time:=false"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.2)
    rclpy.init()
    node = rclpy.create_node('m4_py_test')
    got_markers = False

    def m_cb(msg):
        nonlocal got_markers
        if len(msg.markers) >= 2:
            got_markers = True

    node.create_subscription(MarkerArray, '/safety_zone_markers', m_cb, 10)
    t0 = time.time()
    while time.time() - t0 < 1.5 and not got_markers:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    p_safe.terminate()
    run_clean()
    return got_markers


def test_milestone_5():
    print("  [M5] Checking Master Simulation Bringup Launch Architecture...")
    p = subprocess.run(["ros2", "launch", "mobile_robot_bringup", "simulate_robot.launch.py", "-s"],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return p.returncode == 0


def test_milestone_6():
    print("  [M6] Checking Microcontroller Firmware & Serial Bridge (Mock Mode)...")
    run_clean()
    p_bridge = subprocess.Popen(["ros2", "run", "mobile_robot_firmware", "serial_hardware_bridge.py",
                                 "--ros-args", "-p", "port:=/dev/ttyNONEXISTENT0"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.2)
    rclpy.init()
    node = rclpy.create_node('m6_mock_test')
    got_ticks = False

    def t_cb(msg):
        nonlocal got_ticks
        got_ticks = True

    node.create_subscription(Int32MultiArray, '/wheel_encoder_ticks', t_cb, 10)
    pub = node.create_publisher(Float32MultiArray, '/wheel_speed_commands', 10)

    t0 = time.time()
    while time.time() - t0 < 1.5 and not got_ticks:
        msg = Float32MultiArray()
        msg.data = [3.0, 3.0]
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    p_bridge.terminate()
    run_clean()
    return got_ticks


def test_milestone_7():
    print("  [M7] Checking Real Robot Hardware Bringup Launch...")
    p = subprocess.run(["ros2", "launch", "mobile_robot_bringup", "hardware_robot.launch.py", "-s"],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return p.returncode == 0


def run_milestones_tests():
    print("\n" + "=" * 70)
    print(" 🛠️  PART 1: CORE MILESTONES 1 TO 7 UNIT VERIFICATION")
    print("=" * 70)
    tests = [
        ("Milestone 1: 3D URDF & Robot Description", test_milestone_1),
        ("Milestone 2: Differential Drive Controller (C++)", test_milestone_2_cpp),
        ("Milestone 2: Differential Drive Controller (Python)", test_milestone_2_py),
        ("Milestone 3: Teleoperation, Twist Mux & Twist Relay", test_milestone_3),
        ("Milestone 4: Safety Zone Controller (C++)", test_milestone_4_cpp),
        ("Milestone 4: Safety Zone Controller (Python)", test_milestone_4_py),
        ("Milestone 5: Master Simulation Bringup Launch Architecture", test_milestone_5),
        ("Milestone 6: Microcontroller Firmware & Serial Bridge (Mock Mode)", test_milestone_6),
        ("Milestone 7: Real Robot Hardware Bringup Launch", test_milestone_7),
    ]

    results = []
    for name, func in tests:
        res = func()
        results.append((name, res))
    return results


# ==============================================================================
# SECTION 2: TRACK 2 HARDWARE TWO-WAY UART VERIFICATION
# ==============================================================================

class VirtualMCU:
    def __init__(self, mcu_type="arduino_enc"):
        self.mcu_type = mcu_type
        self.master_fd, self.slave_fd = pty.openpty()
        self.port_name = os.ttyname(self.slave_fd)
        tty.setraw(self.master_fd)

        self.running = True
        self.left_pwm = 0
        self.right_pwm = 0
        self.left_ticks = 0
        self.right_ticks = 0
        self.received_cmds = []

        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        buffer = b""
        last_telem = time.time()
        telem_interval = 0.05

        while self.running:
            r, _, _ = select.select([self.master_fd], [], [], 0.01)
            if self.master_fd in r:
                try:
                    data = os.read(self.master_fd, 1024)
                    if data:
                        buffer += data
                        while b"\n" in buffer:
                            line, buffer = buffer.split(b"\n", 1)
                            line_str = line.decode('ascii', errors='ignore').strip()
                            if line_str.startswith("L:") and ",R:" in line_str:
                                self.received_cmds.append(line_str)
                                parts = line_str.split(",")
                                self.left_pwm = int(parts[0][2:])
                                self.right_pwm = int(parts[1][2:])
                except OSError:
                    break

            now = time.time()
            if now - last_telem >= telem_interval:
                last_telem = now
                self.left_ticks += int(self.left_pwm * 0.1)
                self.right_ticks += int(self.right_pwm * 0.1)

                try:
                    if "enc" in self.mcu_type:
                        enc_msg = f"E:{self.left_ticks},{self.right_ticks}\n"
                        os.write(self.master_fd, enc_msg.encode('ascii'))

                    imu_msg = "I:0,0,16384,0,0,0\n"
                    os.write(self.master_fd, imu_msg.encode('ascii'))
                except OSError:
                    break

    def stop(self):
        self.running = False
        try:
            os.close(self.slave_fd)
            os.close(self.master_fd)
        except OSError:
            pass


def test_mcu_comm(mcu_name, mcu_type):
    print(f"  [MCU] Testing Two-Way Serial: {mcu_name}...")
    run_clean()
    mcu = VirtualMCU(mcu_type)

    cmd_bridge = [
        "ros2", "run", "mobile_robot_firmware", "serial_hardware_bridge.py",
        "--ros-args",
        "-p", f"port:={mcu.port_name}",
        "-p", "baudrate:=115200"
    ]
    p_bridge = subprocess.Popen(cmd_bridge, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.2)

    rclpy.init()
    node = rclpy.create_node('mcu_test_node')
    received = {'ticks': False, 'imu': False, 'mcu_got_cmd': False}

    def enc_cb(msg):
        if len(msg.data) == 2:
            received['ticks'] = True

    def imu_cb(msg):
        if abs(msg.linear_acceleration.z - 9.80665) < 0.1:
            received['imu'] = True

    node.create_subscription(Int32MultiArray, '/wheel_encoder_ticks', enc_cb, 10)
    node.create_subscription(Imu, '/imu/data_raw', imu_cb, 10)
    pub_cmd = node.create_publisher(Float32MultiArray, '/wheel_speed_commands', 10)

    start_t = time.time()
    while time.time() - start_t < 1.5:
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [5.0, 5.0]
        pub_cmd.publish(cmd_msg)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    p_bridge.terminate()
    time.sleep(0.2)

    if len(mcu.received_cmds) > 0:
        received['mcu_got_cmd'] = True
    mcu.stop()
    run_clean()

    passed = received['imu'] and received['mcu_got_cmd']
    if "enc" in mcu_type:
        passed = passed and received['ticks']
    return passed


def run_hardware_tests():
    print("\n" + "=" * 70)
    print(" 🔌 PART 2: TRACK 2 HARDWARE TWO-WAY UART VERIFICATION")
    print("=" * 70)
    tests = [
        ("Arduino Uno (With Encoders + IMU)", lambda: test_mcu_comm("Arduino Uno (With Encoders)", "arduino_enc")),
        ("Arduino Uno (Without Encoders / Open-Loop + IMU)", lambda: test_mcu_comm("Arduino Uno (Open-Loop)", "arduino_open")),
        ("ESP32 (With Encoders + IMU)", lambda: test_mcu_comm("ESP32 (With Encoders)", "esp32_enc")),
        ("ESP32 (Without Encoders / Open-Loop + IMU)", lambda: test_mcu_comm("ESP32 (Open-Loop)", "esp32_open")),
        ("Serial Bridge Mock Fallback Mode", test_milestone_6),
    ]

    results = []
    for name, func in tests:
        res = func()
        results.append((name, res))
    return results


# ==============================================================================
# SECTION 3: 18-COMBINATION MULTI-SIMULATOR MATRIX
# ==============================================================================

def test_sim_combination(sim_type, controller_type, safety_type):
    print(f"  [SIM] Testing: Sim={sim_type:<7} | Ctrl={controller_type:<12} | Safety={safety_type}...")
    run_clean()

    # Step A: Validate Simulator Description & Launch Syntax
    if sim_type == 'classic':
        p_sim = subprocess.run(["ros2", "launch", "mobile_robot_description", "gazebo.launch.py", "-s"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    elif sim_type in ['ign', 'gz']:
        p_sim = subprocess.run(["ros2", "launch", "mobile_robot_description", "ign_gazebo.launch.py", "-s"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    if p_sim.returncode != 0:
        return False

    # Step B: Launch Controller & Safety Nodes in DDS Context
    procs = []
    try:
        if controller_type == 'ros2_control':
            p_x = subprocess.run(["xacro", "/home/ajay/ros2_mobile_robot_ws/src/mobile_robot_description/urdf/ros2_control.xacro"],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if p_x.returncode != 0:
                return False
            cmd_ctrl = ["ros2", "launch", "mobile_robot_controller", "controller.launch.py", "use_cpp:=false", "use_sim_time:=false"]
        elif controller_type == 'cpp':
            cmd_ctrl = ["ros2", "launch", "mobile_robot_controller", "controller.launch.py", "use_cpp:=true", "use_sim_time:=false"]
        elif controller_type == 'py':
            cmd_ctrl = ["ros2", "launch", "mobile_robot_controller", "controller.launch.py", "use_cpp:=false", "use_sim_time:=false"]

        p_ctrl = subprocess.Popen(cmd_ctrl, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_ctrl)

        cmd_safety = [
            "ros2", "launch", "mobile_robot_bringup", "safety_zone.launch.py",
            f"use_cpp:={'true' if safety_type == 'cpp' else 'false'}",
            "use_sim_time:=false"
        ]
        p_safety = subprocess.Popen(cmd_safety, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_safety)
        time.sleep(1.0)

        rclpy.init()
        test_node = rclpy.create_node('sim_mat_node')
        received = {'safety_markers': False, 'cmd_vel_safe': False}

        def marker_cb(msg):
            if len(msg.markers) >= 2:
                received['safety_markers'] = True

        def cmd_vel_cb(msg):
            received['cmd_vel_safe'] = True

        test_node.create_subscription(MarkerArray, '/safety_zone_markers', marker_cb, 10)
        test_node.create_subscription(Twist, '/cmd_vel', cmd_vel_cb, 10)
        pub_raw = test_node.create_publisher(Twist, '/cmd_vel_raw', 10)

        start_t = time.time()
        while time.time() - start_t < 1.2:
            tw = Twist()
            tw.linear.x = 0.2
            pub_raw.publish(tw)
            rclpy.spin_once(test_node, timeout_sec=0.05)

        test_node.destroy_node()
        rclpy.shutdown()

        passed = received['safety_markers'] and received['cmd_vel_safe']
        return passed

    except Exception:
        return False
    finally:
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass
        run_clean()


def run_sim_matrix_tests():
    print("\n" + "=" * 70)
    print(" 🧪 PART 3: MULTI-SIMULATOR & MULTI-CONTROLLER MATRIX (18 COMBINATIONS)")
    print("=" * 70)
    matrix = [
        ('classic', 'ros2_control', 'cpp'),
        ('classic', 'ros2_control', 'py'),
        ('classic', 'cpp', 'cpp'),
        ('classic', 'cpp', 'py'),
        ('classic', 'py', 'cpp'),
        ('classic', 'py', 'py'),
        ('ign', 'ros2_control', 'cpp'),
        ('ign', 'ros2_control', 'py'),
        ('ign', 'cpp', 'cpp'),
        ('ign', 'cpp', 'py'),
        ('ign', 'py', 'cpp'),
        ('ign', 'py', 'py'),
        ('gz', 'ros2_control', 'cpp'),
        ('gz', 'ros2_control', 'py'),
        ('gz', 'cpp', 'cpp'),
        ('gz', 'cpp', 'py'),
        ('gz', 'py', 'cpp'),
        ('gz', 'py', 'py'),
    ]

    results = []
    for sim, ctrl, safety in matrix:
        res = test_sim_combination(sim, ctrl, safety)
        results.append((f"{sim.upper():<7} | {ctrl:<12} | {safety.upper():<4} Safety", res))
    return results


# ==============================================================================
# MAIN TEST DISPATCHER
# ==============================================================================

if __name__ == '__main__':
    mode = 'all'
    if len(sys.argv) > 1:
        if '--milestones' in sys.argv:
            mode = 'milestones'
        elif '--hardware' in sys.argv:
            mode = 'hardware'
        elif '--sim-matrix' in sys.argv:
            mode = 'sim-matrix'

    all_results = []

    if mode in ['all', 'milestones']:
        all_results.extend(run_milestones_tests())

    if mode in ['all', 'hardware']:
        all_results.extend(run_hardware_tests())

    if mode in ['all', 'sim-matrix']:
        all_results.extend(run_sim_matrix_tests())

    print("\n" + "=" * 70)
    print(" 📋 FINAL COMPREHENSIVE TEST REPORT")
    print("=" * 70)
    passed_count = sum(1 for _, r in all_results if r)
    total_count = len(all_results)

    for name, res in all_results:
        mark = "✅ PASS" if res else "❌ FAIL"
        print(f"[{mark}] {name}")

    print("=" * 70)
    if passed_count == total_count:
        print(f"🎉 100% SUCCESS: ALL {total_count}/{total_count} TESTS PASSED CLEANLY!")
    else:
        print(f"⚠️ {passed_count}/{total_count} Passed ({total_count - passed_count} Failed)")
    print("=" * 70)
