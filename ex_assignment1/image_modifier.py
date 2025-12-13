#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from ex_assignment1_interfaces.action import DrawBox

import cv2
import time
import cv2.aruco as aruco
import numpy as np

class ImageModifier(Node):

    def __init__(self):
        super().__init__("image_modifier")

        self.bridge = CvBridge()

        # Latest image
        self.latest_img = None
        self.latest_header = None

        # Box-drawing state
        self.draw_box_active = False
        self.draw_box_end_time = 0.0
        
        # --- ArUco configuration ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters_create()
        self.marker_size = 0.125  # meters


        # Timer to republish continuously
        self.create_timer(0.1, self.timer_publish)

        # Subscribe to camera raw image
        self.create_subscription(Image, "/camera/image", self.image_callback, 10)

        # Publisher: output modified raw image
        self.pub = self.create_publisher(Image, "/camera/image_modified", 10)

        # Action server to request drawing a square
        self.action_server = ActionServer(
            self,
            DrawBox,
            "draw_box",
            self.execute_callback
        )

        self.get_logger().info("image_modifier is running (RAW output + persistent box).")

    def image_callback(self, msg):
        """Store the latest image."""
        self.latest_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.latest_header = msg.header

    def publish_raw(self, cv_image):
        """Publish raw image without compression."""
        if self.latest_header is None:
            return
        raw_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        raw_msg.header = self.latest_header
        self.pub.publish(raw_msg)

    def draw_aruco_boundaries(self, img, target_marker_id=None):
        """
        Detect ArUco markers and draw their real boundaries.
        If target_marker_id is given, only draw that marker.
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is None:
            return img

        for i, marker_id in enumerate(ids.flatten()):
            if target_marker_id is not None and marker_id != target_marker_id:
                continue

            marker_corners = corners[i].reshape((4, 2)).astype(int)

            # Draw marker boundary
            cv2.polylines(
                img,
                [marker_corners],
                isClosed=True,
                color=(255, 0, 0),
                thickness=3
            )

            # Marker center
            cx = int(marker_corners[:, 0].mean())
            cy = int(marker_corners[:, 1].mean())

            cv2.putText(
                img,
                f"ID {marker_id}",
                (cx - 20, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        return img


    def timer_publish(self):
        """Continuously republish image, adding box while active."""
        if self.latest_img is None:
            return

        now = time.time()

        # If box drawing is active for 5 seconds
        if self.draw_box_active and now < self.draw_box_end_time:
            img = self.draw_aruco_boundaries(
                self.latest_img.copy(),
                target_marker_id=self.active_marker_id
            )
            self.publish_raw(img)
        else:
            self.draw_box_active = False
            self.publish_raw(self.latest_img)

    async def execute_callback(self, goal_handle):
        """Action callback: activate drawing for 5 seconds."""
        marker_id = goal_handle.request.marker_id

        if self.latest_img is None:
            goal_handle.abort()
            return DrawBox.Result(success=False, message="No image received")

        # Enable 5-second persistent drawing
        self.draw_box_active = True
        self.draw_box_end_time = time.time() + 5.0
        self.active_marker_id = marker_id

        goal_handle.succeed()
        return DrawBox.Result(success=True, message="Drawing box for 5 seconds")

def main():
    rclpy.init()
    node = ImageModifier()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
