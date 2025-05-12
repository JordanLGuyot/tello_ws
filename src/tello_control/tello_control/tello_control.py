#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String

# ────────────────────────────── constants ──────────────────────────────
PI          = 3.14159265359
PI2         = PI * 2.0
DEG_TO_RAD  = PI / 180.0

NO_KEY      = -1

KEY_NUM_0   = 48
KEY_NUM_1   = 49
KEY_NUM_2   = 50
KEY_NUM_3   = 51
KEY_NUM_4   = 52
KEY_NUM_5   = 53
KEY_NUM_6   = 54
KEY_NUM_7   = 55
KEY_NUM_8   = 56
KEY_NUM_9   = 57

KEY_UP      = 82
KEY_DOWN    = 84
KEY_LEFT    = 81
KEY_RIGHT   = 83

KEY_ENTER   = 13
KEY_SPACE   = 32

# Arrow keys in OpenCV come through as two events; these are the second codes.
ARROW_CODES = {KEY_LEFT, KEY_RIGHT, KEY_UP, KEY_DOWN}

class TelloControl(Node):
    """
    Performs basic keyboard tele-operation of a DJI Tello via the
    tello_driver ROS 2 interface.
    """

    def __init__(self) -> None:
        super().__init__('tello_control')

        # keep track of the current velocity
        self.current_vel = Twist()

        qos = 1
        self.pub_vel       = self.create_publisher(Twist,  'control',  qos)
        self.pub_takeoff   = self.create_publisher(Empty,  'takeoff',  qos)
        self.pub_flip      = self.create_publisher(String, 'flip',     qos)
        self.pub_land      = self.create_publisher(Empty,  'land',     qos)
        self.pub_emergency = self.create_publisher(Empty,  'emergency', qos)

        # 15 ms ≃ 66 Hz – matches cv2.waitKey(15)
        self.timer = self.create_timer(0.015, self.timer_cb)

        # For change detection (unused, but kept for parity with C++)
        self.last_key: int = NO_KEY

        # Minimal dummy image so OpenCV creates a window
        self.blank_image = cv2.cvtColor(
            cv2.UMat(100, 100, cv2.CV_8UC3), cv2.COLOR_BGR2RGB
        )
        cv2.namedWindow('Tello', cv2.WINDOW_AUTOSIZE)

        self.get_logger().info('TelloControl node started (ROS 2 Humble / Python)')

    # ─────────────────────────── helpers ──────────────────────────── #

    def send_empty(self, publisher):
        """Utility to publish an Empty message."""
        publisher.publish(Empty())

    # ─────────────────────────── callbacks ─────────────────────────── #

    def timer_cb(self):
        # Show a dummy window so cv2 can capture keystrokes
        cv2.imshow('Tello', self.blank_image)
        key = cv2.waitKey(1) & 0xFF  # Mask to 8 bits

        if key != NO_KEY:
            self.process_key(key)

        self.last_key = key  # Keep for diffing/debug if desired

    # ──────────────────────── tele‑op logic ───────────────────────── #

    def process_key(self, key: int):
        """Map key-presses to drone commands."""
        # Take‑off / land / flip / emergency
        if key == ord('t'):
            self.get_logger().info('Take-off requested')
            self.send_empty(self.pub_takeoff)
            return

        if key == ord('l'):
            self.get_logger().info('Land requested')
            self.send_empty(self.pub_land)
            return

        if key == ord('f'):
            self.get_logger().info('Flip requested')
            flip_msg = String()
            flip_msg.data = 'f'
            self.pub_flip.publish(flip_msg)
            return

        if key == ord('e'):
            self.get_logger().warn('!! EMERGENCY STOP !!')
            self.send_empty(self.pub_emergency)
            return

        # Otherwise, treat as manual velocity control
        self.manual_control(key)

    def manual_control(self, key: int):
        """Increment or decrement the stored Twist, then publish it."""
        step = 10.0

        # horizontal yaw
        if key == KEY_LEFT:
            self.current_vel.angular.z -= step
        elif key == KEY_RIGHT:
            self.current_vel.angular.z += step

        # vertical axis (up/down)
        elif key == KEY_UP:
            self.current_vel.linear.z += step
        elif key == KEY_DOWN:
            self.current_vel.linear.z -= step

        # forward/back (y)
        elif key == ord('w'):
            self.current_vel.linear.y += step
        elif key == ord('s'):
            self.current_vel.linear.y -= step

        # left/right strafe (x)
        elif key == ord('a'):
            self.current_vel.linear.x -= step
        elif key == ord('d'):
            self.current_vel.linear.x += step

        # stop
        elif key == ord('x'):
            self.current_vel = Twist()          # reset all axes

        else:
            return  # unhandled key

        # optionally clamp velocities here if you want a max speed
        # e.g.
        # max_lin = 100.0
        # self.current_vel.linear.x = max(min(self.current_vel.linear.x, max_lin), -max_lin)
        # … same for other axes …

        self.pub_vel.publish(self.current_vel)

# ─────────────────────────────── main ─────────────────────────────── #

def main():
    rclpy.init()
    node = TelloControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the window closes cleanly
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
