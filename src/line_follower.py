#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class WhiteLineFollowerFromCamera:
    def __init__(self):
        rospy.init_node("white_line_follower_from_camera")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.k_y = 0.01
        self.target_x = 0.4

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge exception: %s", e)
            return

        # 白色抽出（HSVで）
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # エッジ検出 → ハフ変換
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)

        height, width = mask.shape
        center_x = width // 2
        closest_left_x = -1
        closest_right_x = width + 1

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_avg = (x1 + x2) // 2
                y_avg = (y1 + y2) // 2
                if y_avg > height // 3:
                    if x_avg < center_x and x_avg > closest_left_x:
                        closest_left_x = x_avg
                    elif x_avg > center_x and x_avg < closest_right_x:
                        closest_right_x = x_avg

        # 移動制御指令
        twist = Twist()
        twist.linear.x = self.target_x

        if closest_left_x >= 0 and closest_right_x <= width:
            mid = (closest_left_x + closest_right_x) // 2
            error = center_x - mid
            twist.linear.y = self.k_y * error
            rospy.loginfo("Tracking midline. Error: %d", error)
        else:
            rospy.logwarn("Could not find both left and right lines. y=0")
            twist.linear.y = 0.0

        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    try:
        WhiteLineFollowerFromCamera()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
