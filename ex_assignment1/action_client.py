#!/usr/bin/env python3
"""
ServoMarker Action Client Node.

This module implements a ROS 2 action client that sequentially sends
ServoMarker goals to the corresponding action server.

Workflow:
    1. Load detected ArUco marker IDs from a YAML file.
    2. Sort the marker IDs.
    3. Send ServoMarker goals one-by-one.
    4. Wait for each goal to complete before sending the next.
    5. Stop automatically when all markers are processed.

This client is designed to work together with:
    - ArucoObserver (marker detection)
    - ServoMarker action server (robot servoing behavior)

The node follows a **sequential action execution pattern**, ensuring
safe and deterministic robot motion.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import yaml
from ex_assignment1_interfaces.action import ServoMarker


class ServoMarkerClient(Node):
    """
    ROS 2 Action Client for the ServoMarker action.

    This node:
    - Loads a list of detected ArUco marker IDs from a YAML file
    - Sends ServoMarker goals sequentially
    - Waits for each goal result before proceeding to the next

    Attributes
    ----------
    client : ActionClient
        Action client connected to the `servo_marker` action server.
    yaml_path : str
        Absolute path to the YAML file containing detected marker IDs.
    marker_ids : list[int]
        Sorted list of marker IDs to be processed.
    current_index : int
        Index of the currently active marker in `marker_ids`.
    goal_in_progress : bool
        Flag indicating whether a goal is currently active.
    """
    
    def __init__(self):
        """
        Initialize the ServoMarker action client node.

        - Creates the action client.
        - Loads marker IDs from YAML.
        - Waits for the action server.
        - Starts sending goals sequentially.
        """
        super().__init__('servo_marker_client')

        self.client = ActionClient(self, ServoMarker, 'servo_marker')
        self.yaml_path = "/home/shady/ros2_ws/src/detections.yaml"

        # Storage for sequential sending
        self.marker_ids = []
        self.current_index = 0
        self.goal_in_progress = False

        self.get_logger().info("Action Client READY")

        # Load IDs and start workflow
        self.load_and_start()

    def load_yaml(self):
        """
        Load detected marker IDs from a YAML file.

        Returns
        -------
        list[int]
            List of detected ArUco marker IDs.
        """
        with open(self.yaml_path, "r") as f:
            return yaml.safe_load(f)

    def load_and_start(self):
        """
        Load marker IDs and start the action execution workflow.

        - Reads marker IDs from YAML.
        - Sorts the IDs.
        - Waits for the ServoMarker action server.
        - Sends the first goal.
        """
        self.marker_ids = self.load_yaml()
        self.marker_ids.sort()

        self.get_logger().info(f"Loaded {len(self.marker_ids)} marker IDs")

        # Wait for server then start sending sequentially
        self.client.wait_for_server()
        self.send_next_goal()

    # -------------------------------------
    # Send next marker goal (sequential)
    # -------------------------------------
    def send_next_goal(self):
        """
        Send the next ServoMarker goal in sequence.

        This method:
        - Checks if a goal is already active.
        - Stops if all markers have been processed.
        - Sends the next marker ID as a goal.
        """
        if self.goal_in_progress:
            return

        if self.current_index >= len(self.marker_ids):
            self.get_logger().info("🎉 All markers processed.")
            return

        marker_id = self.marker_ids[self.current_index]

        goal = ServoMarker.Goal()
        goal.marker_id = marker_id

        self.get_logger().info(f"➡️ Sending marker {marker_id} to server...")

        self.goal_in_progress = True

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # -------------------------------------
    # Goal accepted / rejected
    # -------------------------------------
    def goal_response_callback(self, future):
        """
        Callback executed when the action server responds to a goal request.

        Parameters
        ----------
        future : rclpy.task.Future
            Future containing the goal handle returned by the action server.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"❌ Marker {self.marker_ids[self.current_index]} rejected.")
            self.goal_in_progress = False
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info("✅ Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # -------------------------------------
    # Goal completed — send next one
    # -------------------------------------
    def result_callback(self, future):
        """
        Callback executed when the action server finishes executing a goal.

        Parameters
        ----------
        future : rclpy.task.Future
            Future containing the action result.
        """
        result = future.result().result
        marker_id = self.marker_ids[self.current_index]

        self.get_logger().info(f"📨 Server Response for marker {marker_id}: {result.message}")

        # Mark goal finished
        self.goal_in_progress = False

        # Move to next ID
        self.current_index += 1

        # Send next goal
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = ServoMarkerClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
