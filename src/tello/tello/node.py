#!/usr/bin/env python3
# ROS 2 Humble version of the Tello driver node
#
# Tested on Ubuntu 22.04 / ROS 2 Humble with Python 3.10
# Dependencies: djitellopy, opencv‑python, PyYAML, cv_bridge, tello_msg package

import math
import threading
import time
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import (
    Image,
    Imu,
    BatteryState,
    Temperature,
    CameraInfo,
)
from nav_msgs.msg import Odometry
from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig
import tf2_ros
from cv_bridge import CvBridge
from djitellopy import Tello
from ament_index_python.packages import get_package_share_directory
import yaml

# ------------------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------------------

def euler_to_quaternion(r: List[float]):
    """Convert [yaw, pitch, roll] to (x, y, z, w)."""
    yaw, pitch, roll = r
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) \
        - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) \
        + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) \
        - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) \
        + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]

# ------------------------------------------------------------------------------
# Main driver class
# ------------------------------------------------------------------------------

class TelloNode:
    """
    Lightweight ROS 2 wrapper around djitellopy’s Tello object.
    """

    def __init__(self, node: Node):
        self.node = node

        # ───── parameters ────────────────────────────────────────────────────
        self.connect_timeout = node.declare_parameter(
            "connect_timeout", 10.0
        ).value
        self.tello_ip = node.declare_parameter(
            "tello_ip", "192.168.10.1"
        ).value
        self.tf_base = node.declare_parameter("tf_base", "map").value
        self.tf_drone = node.declare_parameter("tf_drone", "drone").value
        self.tf_pub = node.declare_parameter("tf_pub", False).value
        self.camera_info_file = node.declare_parameter(
            "camera_info_file", ""
        ).value

        # ───── camera info ───────────────────────────────────────────────────
        if not self.camera_info_file:
            share_dir = get_package_share_directory("tello")
            self.camera_info_file = f"{share_dir}/ost.yaml"

        with open(self.camera_info_file, "r") as f:
            cam_data = yaml.safe_load(f)

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.height = cam_data["image_height"]
        self.camera_info_msg.width = cam_data["image_width"]
        self.camera_info_msg.distortion_model = cam_data["distortion_model"]
        self.camera_info_msg.d = cam_data["distortion_coefficients"]["data"]
        self.camera_info_msg.k = cam_data["camera_matrix"]["data"]
        self.camera_info_msg.r = cam_data["rectification_matrix"]["data"]
        self.camera_info_msg.p = cam_data["projection_matrix"]["data"]

        # ───── connect to drone ──────────────────────────────────────────────
        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connect_timeout)

        self.node.get_logger().info("Connecting to Tello …")
        self.tello = Tello()
        self.tello.connect()
        self.node.get_logger().info("✓ connected")

        # ───── comms setup ───────────────────────────────────────────────────
        self.bridge = CvBridge()
        self._init_qos()
        self._create_publishers()
        self._create_subscribers()

        # ───── background tasks ──────────────────────────────────────────────
        self.start_video_capture()
        self.start_tello_status()
        self.start_tello_odom()

        self.node.get_logger().info("Tello driver ready ✔")

    # ------------------------------------------------------------------ QoS --

    def _init_qos(self):
        self.qos_default = QoSProfile(depth=10)
        # For high‑bandwidth camera topics use best‑effort
        self.qos_image = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

    # ------------------------------------------------------------ publishers --

    def _create_publishers(self):
        self.pub_image_raw = self.node.create_publisher(
            Image, "image_raw", self.qos_image
        )
        self.pub_camera_info = self.node.create_publisher(
            CameraInfo, "camera_info", self.qos_default
        )
        self.pub_status = self.node.create_publisher(
            TelloStatus, "status", self.qos_default
        )
        self.pub_id = self.node.create_publisher(
            TelloID, "id", self.qos_default
        )
        self.pub_imu = self.node.create_publisher(
            Imu, "imu", self.qos_default
        )
        self.pub_battery = self.node.create_publisher(
            BatteryState, "battery", self.qos_default
        )
        self.pub_temperature = self.node.create_publisher(
            Temperature, "temperature", self.qos_default
        )
        self.pub_odom = self.node.create_publisher(
            Odometry, "odom", self.qos_default
        )

        if self.tf_pub:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)

    # ----------------------------------------------------------- subscribers --

    def _create_subscribers(self):
        self.node.create_subscription(
            Empty, "emergency", self.cb_emergency, self.qos_default
        )
        self.node.create_subscription(
            Empty, "takeoff", self.cb_takeoff, self.qos_default
        )
        self.node.create_subscription(
            Empty, "land", self.cb_land, self.qos_default
        )
        self.node.create_subscription(
            Twist, "control", self.cb_control, self.qos_default
        )
        self.node.create_subscription(
            String, "flip", self.cb_flip, self.qos_default
        )
        self.node.create_subscription(
            TelloWifiConfig,
            "wifi_config",
            self.cb_wifi_config,
            self.qos_default,
        )

    # ------------------------------------------------ background publishers --

    def start_tello_odom(self, dt: float = 0.1):
        def _loop():
            while rclpy.ok():
                now = self.node.get_clock().now().to_msg()

                # ── TF
                if self.tf_pub:
                    t = TransformStamped()
                    t.header.stamp = now
                    t.header.frame_id = self.tf_base
                    t.child_frame_id = self.tf_drone
                    t.transform.translation.z = self.tello.get_barometer() / 100.0
                    self.tf_broadcaster.sendTransform(t)

                # ── IMU
                if self.pub_imu.get_subscription_count() > 0:
                    q = euler_to_quaternion(
                        [
                            math.radians(self.tello.get_yaw()),
                            math.radians(self.tello.get_pitch()),
                            math.radians(self.tello.get_roll()),
                        ]
                    )
                    imu_msg = Imu()
                    imu_msg.header.stamp = now
                    imu_msg.header.frame_id = self.tf_drone
                    imu_msg.linear_acceleration.x = self.tello.get_acceleration_x() / 100.0
                    imu_msg.linear_acceleration.y = self.tello.get_acceleration_y() / 100.0
                    imu_msg.linear_acceleration.z = self.tello.get_acceleration_z() / 100.0
                    imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = q
                    self.pub_imu.publish(imu_msg)

                # ── Odom
                if self.pub_odom.get_subscription_count() > 0:
                    q = euler_to_quaternion(
                        [
                            math.radians(self.tello.get_yaw()),
                            math.radians(self.tello.get_pitch()),
                            math.radians(self.tello.get_roll()),
                        ]
                    )
                    odom = Odometry()
                    odom.header.stamp = now
                    odom.header.frame_id = self.tf_base
                    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
                    odom.twist.twist.linear.x = self.tello.get_speed_x() / 100.0
                    odom.twist.twist.linear.y = self.tello.get_speed_y() / 100.0
                    odom.twist.twist.linear.z = self.tello.get_speed_z() / 100.0
                    self.pub_odom.publish(odom)

                time.sleep(dt)

        threading.Thread(target=_loop, daemon=True).start()

    def start_tello_status(self, dt: float = 0.5):
        def _loop():
            while rclpy.ok():
                # Battery
                if self.pub_battery.get_subscription_count() > 0:
                    batt = BatteryState()
                    batt.header.frame_id = self.tf_drone
                    batt.percentage = float(self.tello.get_battery())
                    batt.voltage = 3.8
                    batt.design_capacity = 1.1
                    batt.present = True
                    batt.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
                    batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                    self.pub_battery.publish(batt)

                # Temperature
                if self.pub_temperature.get_subscription_count() > 0:
                    temp = Temperature()
                    temp.header.frame_id = self.tf_drone
                    temp.temperature = self.tello.get_temperature()
                    self.pub_temperature.publish(temp)

                # Status & ID
                if self.pub_status.get_subscription_count() > 0:
                    s = TelloStatus()
                    s.acceleration.x = self.tello.get_acceleration_x()
                    s.acceleration.y = self.tello.get_acceleration_y()
                    s.acceleration.z = self.tello.get_acceleration_z()
                    s.speed.x = float(self.tello.get_speed_x())
                    s.speed.y = float(self.tello.get_speed_y())
                    s.speed.z = float(self.tello.get_speed_z())
                    s.pitch = self.tello.get_pitch()
                    s.roll = self.tello.get_roll()
                    s.yaw = self.tello.get_yaw()
                    s.barometer = int(self.tello.get_barometer())
                    s.distance_tof = self.tello.get_distance_tof()
                    s.fligth_time = self.tello.get_flight_time()
                    s.battery = self.tello.get_battery()
                    s.highest_temperature = self.tello.get_highest_temperature()
                    s.lowest_temperature = self.tello.get_lowest_temperature()
                    s.temperature = self.tello.get_temperature()
                    s.wifi_snr = self.tello.query_wifi_signal_noise_ratio()
                    self.pub_status.publish(s)

                if self.pub_id.get_subscription_count() > 0:
                    ident = TelloID()
                    ident.sdk_version = self.tello.query_sdk_version()
                    ident.serial_number = self.tello.query_serial_number()
                    self.pub_id.publish(ident)

                if self.pub_camera_info.get_subscription_count() > 0:
                    self.camera_info_msg.header.stamp = self.node.get_clock().now().to_msg()
                    self.pub_camera_info.publish(self.camera_info_msg)

                time.sleep(dt)

        threading.Thread(target=_loop, daemon=True).start()

    def start_video_capture(self, dt: float = 1.0 / 30.0):
        self.tello.streamon()
        frame_read = self.tello.get_frame_read()

        def _loop():
            while rclpy.ok():
                msg = self.bridge.cv2_to_imgmsg(np.array(frame_read.frame), "bgr8")
                msg.header.frame_id = self.tf_drone
                msg.header.stamp = self.node.get_clock().now().to_msg()
                self.pub_image_raw.publish(msg)
                time.sleep(dt)

        threading.Thread(target=_loop, daemon=True).start()

    # ---------------------------------------------------------------- IRCB --

    def cb_emergency(self, _):
        self.tello.emergency()

    def cb_takeoff(self, _):
        self.tello.takeoff()

    def cb_land(self, _):
        self.tello.land()

    def cb_control(self, msg: Twist):
        self.tello.send_rc_control(
            int(msg.linear.x),
            int(msg.linear.y),
            int(msg.linear.z),
            int(msg.angular.z),
        )

    def cb_wifi_config(self, msg):
        self.tello.set_wifi_credentials(msg.ssid, msg.password)

    def cb_flip(self, msg: String):
        self.tello.flip(msg.data)

# ------------------------------------------------------------------------------
# main
# ------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tello")
    tello_driver = TelloNode(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        tello_driver.tello.end()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
