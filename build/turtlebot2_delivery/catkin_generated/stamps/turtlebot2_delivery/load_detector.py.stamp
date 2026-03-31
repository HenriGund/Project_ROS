#!/usr/bin/env python3
"""
load_detector.py
================
Detects whether a medication box is present on the robot tray
using the Kinect RGB image and OpenCV colour-blob detection.

How it works:
  1. Subscribes to /camera/rgb/image_raw (Kinect RGB stream).
  2. Converts each frame to HSV colour space.
  3. Thresholds for the box colour (blue by default – adjust HSV range).
  4. If the coloured area exceeds PIXEL_THRESHOLD, box is considered present.
  5. Publishes True/False to /load_status (std_msgs/Bool).

Tuning:
  - Adjust HSV_LOWER / HSV_UPPER to match the actual box colour.
  - Adjust PIXEL_THRESHOLD for your camera distance and box size.
  - Set DEBUG_VIEW to True to open a cv2 window (requires display).

Topics:
  Subscribed:  /camera/rgb/image_raw  (sensor_msgs/Image)   – Kinect RGB
               /mission_state         (std_msgs/String)     – gating
  Published:   /load_status           (std_msgs/Bool)
               /load_detector/debug_image  (sensor_msgs/Image)  – optional
"""

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError


# ── Tuning parameters ──────────────────────────────────────────────────────────
# HSV range for the medicine box colour.
# Default: blue box (H 100-130, S 80-255, V 50-255).
# Use the helper script in the README (Phase 3) to find values for your box.
HSV_LOWER = np.array([100,  80,  50], dtype=np.uint8)
HSV_UPPER = np.array([130, 255, 255], dtype=np.uint8)

# Minimum number of pixels in the colour mask to count as "box present".
PIXEL_THRESHOLD = 3000

# Set True to open a debug window (needs X11/display).
DEBUG_VIEW = False

# Only process frames when the robot is in these states (saves CPU).
ACTIVE_STATES = {'WAITING_LOAD', 'DELIVERING'}

# Kinect RGB topic
IMAGE_TOPIC = '/camera/rgb/image_raw'
# ──────────────────────────────────────────────────────────────────────────────


class LoadDetector:
    def __init__(self):
        rospy.init_node('load_detector', anonymous=False)

        self.bridge = CvBridge()
        self.current_state = 'IDLE'
        self._last_detected = None  # avoids redundant publishes

        self.pub       = rospy.Publisher('/load_status',             Bool,  queue_size=1, latch=True)
        self.debug_pub = rospy.Publisher('/load_detector/debug_image', Image, queue_size=1)

        rospy.Subscriber(IMAGE_TOPIC,      Image,  self.on_image)
        rospy.Subscriber('/mission_state', String, self.on_state)

        rospy.loginfo('[LoadDetector] Subscribing to %s', IMAGE_TOPIC)
        rospy.loginfo('[LoadDetector] HSV lower=%s upper=%s threshold=%d',
                      HSV_LOWER, HSV_UPPER, PIXEL_THRESHOLD)
        rospy.spin()

    def on_state(self, msg):
        self.current_state = msg.data

    def on_image(self, msg):
        if self.current_state not in ACTIVE_STATES:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn('[LoadDetector] CvBridge error: %s', e)
            return

        detected = self.detect_box(frame)

        if detected != self._last_detected:
            self._last_detected = detected
            self.pub.publish(Bool(data=detected))
            rospy.loginfo('[LoadDetector] Box %s', 'DETECTED' if detected else 'REMOVED')

        if DEBUG_VIEW:
            cv2.imshow('LoadDetector', frame)
            cv2.waitKey(1)

    def detect_box(self, frame):
        """Return True if box-coloured blob area exceeds PIXEL_THRESHOLD."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        pixel_count = cv2.countNonZero(mask)

        if self.debug_pub.get_num_connections() > 0:
            debug = frame.copy()
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(debug, contours, -1, (0, 255, 0), 2)
            label = f'BOX: {"YES" if pixel_count > PIXEL_THRESHOLD else "NO"} ({pixel_count}px)'
            cv2.putText(debug, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))
            except CvBridgeError:
                pass

        return pixel_count > PIXEL_THRESHOLD


if __name__ == '__main__':
    try:
        LoadDetector()
    except rospy.ROSInterruptException:
        pass
