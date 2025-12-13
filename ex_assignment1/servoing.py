"""
Servoing Logic for the ServoMarker Action.

This module contains the **core execution logic** of the ServoMarker
action, implemented as a standalone `execute_callback` function.

Design Rationale
----------------
The servoing behavior is separated from the ROS 2 node definition
(`action_server.py`) to:

- Reduce the size and complexity of the action server file
- Clearly isolate the **state machine logic**
- Improve readability and maintainability
- Allow independent testing and reasoning about behavior

This module does **not** define a ROS node.
Instead, it operates on the action server instance via `self`,
which is dynamically bound in `action_server.py`.
"""

import rclpy
from ex_assignment1_interfaces.action import ServoMarker

import time
import math
from geometry_msgs.msg import Twist


def execute_callback(self, goal_handle: ServoMarker.Goal):
    """
    Execute the ServoMarker action goal.

    This function implements a **deterministic state machine** that
    drives the robot toward a specific ArUco marker, pauses for
    visualization, and then retreats safely.

    State Machine Overview
    ----------------------
    The action progresses through the following states:

    1. SEARCHING
       - Rotate in place until the target marker becomes visible.

    2. CENTERING
       - Rotate the robot to align the marker with the camera center.

    3. FORWARD
       - Move forward toward the marker while monitoring laser safety.

    4. PAUSE
       - Stop for a fixed duration and trigger the DrawBox action.

    5. BACKWARD
       - Retreat a fixed distance away from the marker.

    The action terminates successfully once the BACKWARD state completes.

    Parameters
    ----------
    self : ServoMarkerServer
        Instance of the ServoMarker action server node.
        This object provides publishers, sensor state, and helper methods
        imported from `callbacks.py`.

    goal_handle : rclpy.action.server.ServerGoalHandle
        Handle associated with the incoming ServoMarker action goal.

    Returns
    -------
    ServoMarker.Result
        Action result indicating success or failure.
    """
    marker_id = goal_handle.request.marker_id
    self.target_marker_id = int(marker_id)

    self.get_logger().info(
        f"🚀 GOAL RECEIVED — servo to marker {marker_id}"
    )

    # Reset per-goal state
    self._reset_state()
    self.drawbox_done = False
    self.state = "SEARCHING"

    loop_dt = 0.05  # 20 Hz control loop

    try:
        while rclpy.ok():

            # ------------------------------------------------------
            # Handle cancel requests
            # ------------------------------------------------------
            if goal_handle.is_cancel_requested:
                self.get_logger().info("⛔ Goal cancel requested.")
                self._stop_robot()
                goal_handle.canceled()

                result = ServoMarker.Result()
                result.success = False
                result.message = (
                    f"Servo to marker {marker_id} canceled."
                )

                self._clear_goal()
                return result

            twist = Twist()
            now = time.time()

            # ======================================================
            # STATE 1 — SEARCHING
            # ======================================================
            if self.state == "SEARCHING":
                if not self.marker_seen:
                    twist.angular.z = self.search_spin_speed
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    time.sleep(loop_dt)
                    continue

                if self.marker_seen and self.marker_x is not None:
                    self.state = "CENTERING"
                    continue

            # Wait until valid marker pose is available
            if self.marker_x is None or self.marker_z is None:
                time.sleep(loop_dt)
                continue

            mx = self.marker_x
            mz = self.marker_z
            angle_error = math.atan2(mx, mz)
            angle_threshold = 0.1

            # ======================================================
            # STATE 2 — CENTERING
            # ======================================================
            if self.state == "CENTERING":
                if abs(angle_error) < angle_threshold:
                    self.get_logger().info("✅ Marker centered")
                    self.forward_start_z = mz
                    self.state = "FORWARD"
                    continue

                if abs(angle_error) < 0.2:
                    twist.angular.z = (
                        -self.servo_angular_gain_fine * angle_error
                    )
                else:
                    twist.angular.z = (
                        -self.servo_angular_gain_coarse * angle_error
                    )

                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                time.sleep(loop_dt)
                continue

            # ======================================================
            # STATE 3 — FORWARD
            # ======================================================
            if self.state == "FORWARD":
                safe_to_move = (
                    self.laser_front > self.laser_stop_threshold
                )

                if mz > self.forward_distance_threshold and safe_to_move:
                    twist.linear.x = self.forward_speed
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)

                    self.get_logger().info(
                        f"⬆️ Moving forward | "
                        f"Marker={mz:.2f} | "
                        f"Laser={self.laser_front:.2f}"
                    )

                    time.sleep(loop_dt)
                    continue

                # Stop and transition to pause
                self._stop_robot()
                self.pause_start_time = now
                self.backward_target = mz + 0.5

                reason = (
                    "Laser obstacle"
                    if not safe_to_move
                    else "Reached marker threshold"
                )

                self.get_logger().info(
                    f"⏸️ Pausing 5 sec. Reason: {reason}"
                )

                if not self.drawbox_done:
                    self._send_draw_request(marker_id)
                    self.drawbox_done = True

                self.state = "PAUSE"
                time.sleep(loop_dt)
                continue

            # ======================================================
            # STATE 4 — PAUSE
            # ======================================================
            if self.state == "PAUSE":
                if now - self.pause_start_time < 5.0:
                    self._stop_robot()
                    time.sleep(loop_dt)
                    continue

                self.backward_start_z = mz
                self.state = "BACKWARD"
                continue

            # ======================================================
            # STATE 5 — BACKWARD
            # ======================================================
            if self.state == "BACKWARD":
                if mz < self.backward_target:
                    twist.linear.x = self.backward_speed
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)

                    self.get_logger().info(
                        f"⬇️ Backing up | "
                        f"Now={mz:.2f} | "
                        f"Target={self.backward_target:.2f}"
                    )

                    time.sleep(loop_dt)
                    continue

                # Goal complete
                self._stop_robot()
                self.get_logger().info(
                    "🏁 Returned safely. Servo action complete."
                )

                goal_handle.succeed()

                result = ServoMarker.Result()
                result.success = True
                result.message = (
                    f"Servo complete for marker {marker_id}"
                )

                self._clear_goal()
                return result

            # --------------------------------------------------
            # Fallback safety stop (should never occur)
            # --------------------------------------------------
            self._stop_robot()
            time.sleep(loop_dt)

    except Exception as e:
        self.get_logger().error(
            f"❌ Exception during servo execution: {e}"
        )

        self._stop_robot()
        goal_handle.abort()

        result = ServoMarker.Result()
        result.success = False
        result.message = (
            f"Exception during servo to marker {marker_id}: {e}"
        )

        self._clear_goal()
        return result
