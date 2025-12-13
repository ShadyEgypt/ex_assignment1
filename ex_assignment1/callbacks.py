"""
Shared Callbacks and Helper Functions for ServoMarker Action.

This module groups together **sensor callbacks** and **utility helpers**
used by the ServoMarker action server.

Design Rationale
----------------
The functions in this file are intentionally separated from
`action_server.py` to:

- Reduce the size and complexity of the action server file
- Avoid duplicating logic inside the servoing state machine
- Improve clarity by isolating perception, safety, and helper behaviors
- Allow reuse across different execution contexts if needed

All functions operate on the action server instance via `self`
and are dynamically bound in `action_server.py`.
"""

from sensor_msgs.msg import LaserScan
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist
from ex_assignment1_interfaces.action import DrawBox


# ------------------------------------------------------------------
# LASER CALLBACK
# ------------------------------------------------------------------
def laser_callback(self, msg: LaserScan):
    """
    Process laser scan data to compute the minimum frontal obstacle distance.

    This callback extracts a small angular window centered in front of the robot
    and computes the minimum valid distance. The result is stored in
    `self.laser_front` and continuously updated.

    This value is used by the servoing state machine to:
    - Prevent forward motion when an obstacle is too close
    - Trigger an early transition from FORWARD to PAUSE if needed

    Parameters
    ----------
    msg : sensor_msgs.msg.LaserScan
        Incoming laser scan message.
    """
    if not msg.ranges:
        self.laser_front = float("inf")
        return

    center = len(msg.ranges) // 2
    window = 8

    start = max(0, center - window)
    end = min(len(msg.ranges), center + window)

    front_window = msg.ranges[start:end]
    valid_ranges = [r for r in front_window if r > 0.05]

    if valid_ranges:
        self.laser_front = min(valid_ranges)
    else:
        self.laser_front = float("inf")


# ------------------------------------------------------------------
# MARKER CALLBACK
# ------------------------------------------------------------------
def marker_callback(self, msg: ArucoDetection):
    """
    Update the pose of the currently requested ArUco marker.

    This callback scans all detected markers and updates the internal
    marker state **only** if the detected marker matches the
    `target_marker_id`.

    The following fields are updated:
    - `self.marker_x`
    - `self.marker_z`
    - `self.marker_seen`

    Parameters
    ----------
    msg : aruco_opencv_msgs.msg.ArucoDetection
        Message containing detected ArUco markers.
    """
    if self.target_marker_id is None:
        return

    seen = False

    for m in msg.markers:
        if int(m.marker_id) == int(self.target_marker_id):
            position = m.pose.position
            self.marker_x = position.x
            self.marker_z = position.z
            self.marker_seen = True
            seen = True
            break

    if not seen:
        self.marker_seen = False


# ------------------------------------------------------------------
# HELPER: clear goal state
# ------------------------------------------------------------------
def _clear_goal(self):
    """
    Clear all per-goal state variables.

    This helper is called when:
    - A goal completes successfully
    - A goal is canceled
    - A goal fails due to an exception

    It resets the action server to an IDLE state.
    """
    self.target_marker_id = None
    self.state = "IDLE"


# ------------------------------------------------------------------
# HELPER: stop robot motion
# ------------------------------------------------------------------
def _stop_robot(self):
    """
    Immediately stop the robot by publishing zero velocity commands.
    """
    self.cmd_pub.publish(Twist())


# ------------------------------------------------------------------
# HELPER: reset per-goal servoing state
# ------------------------------------------------------------------
def _reset_state(self):
    """
    Reset all state variables required by the servoing state machine.

    This function is called at the beginning of each action goal to
    ensure deterministic and repeatable behavior.
    """
    self.state = "SEARCHING"
    self.marker_x = None
    self.marker_z = None
    self.marker_seen = False

    self.forward_start_z = None
    self.backward_start_z = None
    self.backward_target = None
    self.pause_start_time = None


# ------------------------------------------------------------------
# HELPER: send DrawBox action request
# ------------------------------------------------------------------
def _send_draw_request(self, marker_id: int):
    """
    Send a non-blocking DrawBox action request.

    This helper triggers the DrawBox action server to draw a bounding box
    around the detected ArUco marker for visualization purposes.

    The request is sent in a **fire-and-forget** fashion:
    - The servoing action does not wait for the DrawBox result
    - Execution continues immediately

    Parameters
    ----------
    marker_id : int
        ID of the ArUco marker to be highlighted.
    """
    goal_msg = DrawBox.Goal()
    goal_msg.marker_id = int(marker_id)

    if not self.draw_client.server_is_ready():
        self.get_logger().warn(
            "⚠️ DrawBox action server not ready. Skipping draw request."
        )
        return

    self.get_logger().info(
        f"🖼️ Sending DrawBox goal for marker {marker_id}"
    )

    send_future = self.draw_client.send_goal_async(goal_msg)

    def goal_response_cb(future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("DrawBox goal was rejected.")
            return

        self.get_logger().info("DrawBox goal accepted.")

        result_future = goal_handle.get_result_async()

        def result_cb(result_future):
            result = result_future.result().result
            self.get_logger().info(
                f"DrawBox result: success={result.success} "
                f"message='{result.message}'"
            )

        result_future.add_done_callback(result_cb)

    send_future.add_done_callback(goal_response_cb)
