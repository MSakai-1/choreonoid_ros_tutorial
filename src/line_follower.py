#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class LineFollower:
    def __init__(self):
        rospy.init_node("line_follower")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/go2_description/camera1/image/compressed", CompressedImage, self.image_callback, queue_size=1)
        self.yaw_pub = rospy.Publisher("/yaw_error", Float32, queue_size=1)
        self.visualize = rospy.get_param("~visualize", True)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        # mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # edge & hough
        edges = cv2.Canny(mask, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=10)

        yaw_angles = []

        if lines is not None:
            img_center_x = frame.shape[1] // 2
            line_info = [] 

            for line in lines:
                x1, y1, x2, y2 = line[0]
                mid_x = (x1 + x2) / 2.0
                dist_to_center = abs(mid_x - img_center_x)

                angle_rad = math.atan2(y2 - y1, x2 - x1)
                line_info.append((dist_to_center, angle_rad, (x1, y1, x2, y2)))
                angle_deg = np.rad2deg(angle_rad)
                
            line_info.sort(key=lambda x: x[0])
            selected_lines = line_info[:4]

            yaw_angles = []
            for _, yaw, (x1, y1, x2, y2) in selected_lines:
                yaw_angles.append(yaw)
                if self.visualize:
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
        
        if yaw_angles:
            mean_yaw = np.mean(yaw_angles)
            self.yaw_pub.publish(Float32(data=mean_yaw))
            rospy.loginfo(f"mean_yaw: {math.degrees(mean_yaw):.2f} deg")

        if self.visualize:
            cv2.imshow("line detection", frame)
            cv2.waitKey(1)

if __name__ == "__main__":
    try:
        LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
