"""
ServoMarker Action Server Node.

This module implements the ROS 2 Action Server responsible for executing
the ServoMarker action.

Design philosophy
-----------------
To keep the action server **readable, maintainable, and testable**, the
implementation is intentionally split across multiple modules:

- `action_server.py`
    Defines the ROS 2 node, action server, publishers, subscriptions,
    and concurrency model.

- `servoing.py`
    Contains the **core action execution logic** (state machine),
    implemented as the `execute_callback` function.

- `callbacks.py`
    Contains sensor callbacks (laser and marker detection) and
    reusable helper methods shared across the action lifecycle.

This separation allows the action server node to focus on **ROS wiring**
while delegating behavior and logic to specialized modules.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from aruco_opencv_msgs.msg import ArucoDetection
from ex_assignment1_interfaces.action import ServoMarker, DrawBox

from .servoing import execute_callback
from .callbacks import (
    laser_callback,
    marker_callback,
    _clear_goal,
    _stop_robot,
    _reset_state,
    _send_draw_request,
)


class ServoMarkerServer(Node):
    """
    ROS 2 Action Server for the ServoMarker action.

    This node executes a **servoing behavior** that moves the robot
    toward a specified ArUco marker and then safely retreats.

    The behavior follows a deterministic state machine:
        SEARCHING → CENTERING → FORWARD → PAUSE → BACKWARD → DONE

    Responsibilities
    ----------------
    - Expose the `servo_marker` action interface
    - Subscribe to perception and safety topics
    - Publish velocity commands
    - Coordinate with the DrawBox action
    - Delegate execution logic to modular helper files

    Subscriptions
    -------------
    /aruco_detections : aruco_opencv_msgs.msg.ArucoDetection
        ArUco marker detections used for visual servoing.

    /scan : sensor_msgs.msg.LaserScan
        Laser scan data used for obstacle avoidance.

    Publications
    ------------
    /cmd_vel : geometry_msgs.msg.Twist
        Velocity commands for robot motion.

    Actions
    -------
    Server:
        servo_marker (ex_assignment1_interfaces/action/ServoMarker)

    Client:
        draw_box (ex_assignment1_interfaces/action/DrawBox)
    """

    def __init__(self):
        """
        Initialize the ServoMarker action server node.

        This constructor:
        - Initializes all internal state variables
        - Binds external callback functions
        - Creates publishers and subscriptions
        - Starts the ServoMarker action server
        """
        super().__init__("servo_marker_server")

        # =====================================================
        # Callback group (allows concurrent callbacks)
        # =====================================================
        self.cb_group = ReentrantCallbackGroup()

        # =====================================================
        # ---- STATE VARIABLES (required by external logic) ----
        # =====================================================

        # Marker state
        self.marker_x = None
        self.marker_z = None
        self.marker_seen = False
        self.target_marker_id = None

        # Laser state
        self.laser_front = float("inf")
        self.laser_stop_threshold = 1.0

        # Motion and control parameters
        self.search_spin_speed = 0.3
        self.servo_angular_gain_coarse = 0.5
        self.servo_angular_gain_fine = 0.15
        self.forward_speed = 0.08
        self.backward_speed = -0.08
        self.forward_distance_threshold = 1.0

        # State machine variables
        self.state = "IDLE"
        self.forward_start_z = None
        self.backward_start_z = None
        self.backward_target = None
        self.pause_start_time = None
        self.angle_error = 0.0

        # =====================================================
        # ---- BIND EXTERNAL CALLBACK FUNCTIONS ----------------
        # =====================================================
        # These functions are imported from callbacks.py and
        # servoing.py to keep this file compact and readable.
        self.marker_callback = marker_callback.__get__(self)
        self.laser_callback = laser_callback.__get__(self)

        self._clear_goal = _clear_goal.__get__(self)
        self._stop_robot = _stop_robot.__get__(self)
        self._reset_state = _reset_state.__get__(self)
        self._send_draw_request = _send_draw_request.__get__(self)

        self.execute_callback = execute_callback.__get__(self)

        # =====================================================
        # ---- PUBLISHERS -------------------------------------
        # =====================================================
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # =====================================================
        # ---- SUBSCRIPTIONS ----------------------------------
        # =====================================================
        self.create_subscription(
            ArucoDetection,
            "/aruco_detections",
            self.marker_callback,
            10,
            callback_group=self.cb_group,
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            10,
            callback_group=self.cb_group,
        )

        # =====================================================
        # ---- ACTION CLIENT (DrawBox) ------------------------
        # =====================================================
        self.draw_client = ActionClient(
            self,
            DrawBox,
            "draw_box",
            callback_group=self.cb_group,
        )

        # =====================================================
        # ---- ACTION SERVER (ServoMarker) --------------------
        # =====================================================
        self._action_server = ActionServer(
            self,
            ServoMarker,
            "servo_marker",
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            "✅ ServoMarker Action Server READY "
            "(search → center → forward → draw → back)."
        )


def main(args=None):
    """
    ROS 2 entry point for the ServoMarker action server.

    Uses a MultiThreadedExecutor to allow:
    - Action execution
    - Marker detection callbacks
    - Laser callbacks
    - DrawBox action communication

    to run concurrently without blocking.
    """
    rclpy.init(args=args)
    node = ServoMarkerServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
