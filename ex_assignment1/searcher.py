#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from aruco_opencv_msgs.msg import ArucoDetection
from rclpy.action import ActionClient
from ex_assignment1_interfaces.action import DrawBox
from tf_transformations import euler_from_quaternion
import math
import time


class Searcher(Node):

    def __init__(self):
        super().__init__("searcher")

        # -------------------------------------------------------
        # Declare and read marker_id parameter
        # -------------------------------------------------------
        self.declare_parameter("marker_id", 2)
        self.target_marker_id = self.get_parameter("marker_id").value

        self.get_logger().info(f"🎯 Searcher looking for marker_id = {self.target_marker_id}")

        # -------------------------------------------------------
        # Publishers & Subscriptions
        # -------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(ArucoDetection, "/aruco_detections", self.marker_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)

        # Action client for drawing box
        self.action_client = ActionClient(self, DrawBox, "draw_box")

        # -------------------------------------------------------
        # Internal State
        # -------------------------------------------------------
        self.marker_x = None
        self.marker_z = None
        self.yaw = None
        self.marker_seen = False
        self.sign = 1

        self.laser_front = float("inf")
        self.laser_stop_threshold = 2.0  # stop if obstacle < 2 m

        # Behavior parameters
        self.search_spin_speed = 0.3
        self.servo_angular_gain = 0.2
        self.forward_speed = 0.08
        self.backward_speed = -0.08
        self.forward_distance_threshold = 2.8

        # State machine
        self.done = False
        self.state = "SEARCHING"
        self.forward_start_z = None
        self.backward_target = None
        self.pause_start_time = None
        self.angle_error = 0.0

        # Main loop
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Searcher node started.")

    # -----------------------------------------------------------
    # Marker detection callback
    # -----------------------------------------------------------
    def marker_callback(self, msg: ArucoDetection):
        """Update pose ONLY for the target marker. Reset marker_seen if not found."""
        
        # Assume we will NOT see the target marker in this frame
        target_found = False

        for m in msg.markers:
            if int(m.marker_id) == int(self.target_marker_id):

                # Extract position
                p = m.pose.position
                q = m.pose.orientation
                self.marker_x = p.x
                self.marker_z = p.z
                q_list = [q.x, q.y, q.z, q.w]
                roll, pitch, self.yaw = euler_from_quaternion(q_list)
                # Update state
                self.marker_seen = True
                self.last_seen_time = time.time()
                target_found = True
                break

        # If no matching marker exists in this frame → mark as "not seen"
        if not target_found:
            self.marker_seen = False


    # -----------------------------------------------------------
    # LaserScan callback
    # -----------------------------------------------------------
    def laser_callback(self, msg):
        center = len(msg.ranges) // 2
        window = 10
        seg = msg.ranges[center - window : center + window]
        clean = [r for r in seg if r > 0.05]
        self.laser_front = min(clean) if clean else float("inf")

    # -----------------------------------------------------------
    # Main control logic 
    # -----------------------------------------------------------
    def control_loop(self):
        # If we're done, do nothing (non-blocking)
        if self.state == "DONE":
            return

        twist = Twist()
        now = time.time()

        # ===========================
        # STATE: SEARCHING
        # ===========================
        if not self.marker_seen:
            # Keep searching, don't fall through to other states
            self.state = "SEARCHING"
            twist.angular.z = self.search_spin_speed
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"🔍 Searching for marker {self.target_marker_id}...")
            return

        # At this point marker_seen is True (recently), so we can safely use mx, self.marker_z
        mx_error = self.marker_x
        mz = self.marker_z
        yaw_error = self.yaw
        # Safety: if data is missing, just wait
        if self.marker_x is None or self.marker_z is None:
            return

        # Transition from SEARCHING → APPROACH when we first see the marker
        if self.state == "SEARCHING" and self.marker_seen:
            self.get_logger().info("✅ Marker found! Transitioning to APPROACH state.")
            self.state = "APPROACH"
            return


        # STOP when both errors are tiny
        if abs(self.marker_x) < 0.005:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f"🎯 Perfect alignment! mx={self.marker_x:.4f}, yaw={yaw_error:.3f}"
            )
            self.state = "GOAL_CHECK"
            return

        twist.linear.x = 0.0
        twist.angular.z = 0.0


        # ===========================
        # 1. Coarse rotation using mx
        # ===========================
        if abs(self.marker_x) >= .05:
            sign = 1 if self.marker_x > 0 else -1
            if abs(self.marker_x) < 0.2:
                ang = -0.15  * sign   # fine adjust
            else:
                ang = -0.5  * sign   # coarse adjust

            twist.angular.z = ang
            self.cmd_pub.publish(twist)
            self.get_logger().info(
                f"🔁 Centering marker: mx={self.marker_x:.4f}, ω={ang:.3f}, yaw={yaw_error:.3f}"
            )
            return


        # # ===========================
        # # 3. Move forward when centered enough
        # # ===========================
        # if abs(self.marker_x) < .05 and self.marker_z > 2.0:

        #     # Smooth forward speed based on how close you are
        #     if self.marker_z > 1.5:
        #         fwd = 0.08   # farther → faster
        #     else:
        #         fwd = 0.04   # close → slower for fine control

        #     twist.linear.x = fwd
        #     sign = 1 if self.marker_x > 0 else -1
        #     twist.angular.z = - .1 * sign

        #     self.cmd_pub.publish(twist)
        #     self.get_logger().info(
        #         f"⬆️ Moving forward: self.marker_z={self.marker_z:.2f}, vx={fwd:.2f}"
        #     )
        #     return


        # ===========================
        # STATE: GOAL_CHECK (mx-only)
        # ===========================
        if self.state == "GOAL_CHECK":

            if abs(self.marker_x) <= 0.05:
                self.get_logger().info("🎯 Final check: Goal confirmed.")
                self.pause_start_time = now
                self.state = "PAUSE"
                return
            else:
                self.state = "APPROACH"
                return


        # ===========================
        # STATE: PAUSE
        # ===========================
        if self.state == "PAUSE":
            # Safety: if somehow pause_start_time is None, initialize it
            if self.pause_start_time is None:
                self.pause_start_time = now

            if now - self.pause_start_time < 10.0:
                # Just wait, no movement
                return
            else:
                self.get_logger().info("⏱ Pause finished. Going DONE...")
                self.state = "DONE"
                return

def main(args=None):
    rclpy.init(args=args)
    node = Searcher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

