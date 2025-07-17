#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class VelocityPublisher:
    def __init__(self):
        rospy.init_node('velocity_publisher')
        self.vyaw_gain = 0.5
        self.timeout = 1.0

        self.latest_yaw = 0.0
        self.last_yaw_time = rospy.Time.now()

        rospy.Subscriber("/yaw_error", Float32, self.yaw_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def yaw_callback(self, msg):
        self.latest_yaw = msg.data
        self.last_yaw_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.45
            twist.linear.y = 0.0

            time_since_last = (rospy.Time.now() - self.last_yaw_time).to_sec()
            if time_since_last < self.timeout:
                angular_z = self.vyaw_gain * self.latest_yaw
                twist.angular.z = np.clip(angular_z, -0.25, 0.25)
            else:
                twist.angular.z = 0.0
                rospy.logwarn_throttle(5.0, "No yaw_error received recently. Sending zero angular velocity.")
        
            self.cmd_pub.publish(twist)
            rate.sleep()
        
        

if __name__ == '__main__':
    try:
        controller = VelocityPublisher()
        controller.run()
    except rospy.ROSInterruptException:
        pass
