"""
Aruco Observer Node.

This module implements a ROS 2 node responsible for **environment scanning**
and **ArUco marker discovery**.

The node rotates the robot in place while listening to ArUco detections.
Once a predefined number of unique markers is detected, it:

1. Stops the robot.
2. Stores the detected marker IDs in a YAML file.
3. Launches the ServoMarker action client to process each marker.

This node acts as the **high-level trigger** of the assignment pipeline and
does not perform any servoing or navigation by itself.
"""

import rclpy
from rclpy.node import Node

import yaml
import subprocess
import time

from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection


class ArucoObserver(Node):
    """
    ROS 2 node for scanning and collecting ArUco marker IDs.

    The observer:
    - Rotates the robot in place to scan the environment.
    - Subscribes to `/aruco_detections`.
    - Collects unique marker IDs.
    - Stops scanning after reaching a target count.
    - Saves results to a YAML file.
    - Launches the ServoMarker action client.

    Publications
    ------------
    /cmd_vel : geometry_msgs.msg.Twist
        Velocity commands used to rotate or stop the robot.

    Subscriptions
    -------------
    /aruco_detections : aruco_opencv_msgs.msg.ArucoDetection
        Incoming ArUco marker detections.
    """

    def __init__(self):
        """
        Initialize the ArucoObserver node.

        Sets up publishers, subscriptions, internal state variables,
        and immediately starts the scanning behavior.
        """
        super().__init__('aruco_observer')

        # -----------------------------------------
        # Publishers & Subscribers
        # -----------------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.detection_callback,
            10
        )

        # -----------------------------------------
        # State
        # -----------------------------------------
        self.detected_ids = set()
        self.target_count = 5
        self.yaml_path = "/home/shady/ros2_ws/src/detections.yaml"

        self.scanning = True

        self.get_logger().info(
            "🔄 Aruco Observer started — rotating to scan markers"
        )

        self.spin_robot()

    # ------------------------------------------------
    # Robot motion helpers
    # ------------------------------------------------
    def spin_robot(self):
        """
        Publish angular velocity to rotate the robot in place.

        This method is used while scanning for ArUco markers.
        """
        twist = Twist()
        twist.angular.z = 0.3
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """
        Stop all robot motion by publishing zero velocities.
        """
        self.cmd_pub.publish(Twist())
        self.get_logger().info("🛑 Robot stopped")

    # ------------------------------------------------
    # Detection callback
    # ------------------------------------------------
    def detection_callback(self, msg: ArucoDetection):
        """
        Process incoming ArUco detections.

        This callback:
        - Extracts marker IDs from incoming messages.
        - Tracks unique IDs.
        - Continues rotating while scanning.
        - Stops scanning once the target number of markers is reached.

        Parameters
        ----------
        msg : aruco_opencv_msgs.msg.ArucoDetection
            Message containing detected ArUco markers.
        """
        for m in msg.markers:
            self.detected_ids.add(int(m.marker_id))

        self.get_logger().info(
            f"Detected IDs: {sorted(self.detected_ids)}"
        )

        if self.scanning:
            self.spin_robot()

        if len(self.detected_ids) >= self.target_count and self.scanning:
            self.scanning = False

            self.get_logger().info(
                f"✅ Reached {self.target_count} markers → stopping scan"
            )

            self.stop_robot()
            self.save_yaml()
            self.launch_actions()

            self.get_logger().info(
                "🔚 Observer finished — shutting down"
            )
            rclpy.shutdown()

    # ------------------------------------------------
    # Save detected IDs
    # ------------------------------------------------
    def save_yaml(self):
        """
        Save detected marker IDs to a YAML file.

        The marker IDs are sorted before saving to ensure
        deterministic execution order in the action client.
        """
        sorted_ids = sorted(self.detected_ids)

        with open(self.yaml_path, "w") as f:
            yaml.dump(sorted_ids, f)

        self.get_logger().info(
            f"📁 Saved marker IDs to {self.yaml_path}"
        )

    # ------------------------------------------------
    # Launch action client
    # ------------------------------------------------
    def launch_actions(self):
        """
        Launch the ServoMarker action client as a separate process.

        A short delay is introduced to ensure the system is stable
        before starting action execution.
        """
        self.get_logger().info(
            "🚀 Launching Action Client in 10s..."
        )
        time.sleep(10)

        subprocess.Popen(
            ["ros2", "run", "ex_assignment1", "action_client"]
        )


def main(args=None):
    """
    ROS 2 entry point for the ArucoObserver node.

    Initializes ROS 2, starts the observer node, and spins until shutdown.
    """
    rclpy.init(args=args)
    node = ArucoObserver()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()
