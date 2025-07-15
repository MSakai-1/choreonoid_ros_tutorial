#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class VelocityPublisher:
    def __init__(self):
        rospy.init_node('velocity_publisher')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.run()

    def run(self):
        twist = Twist()
        twist.linear.x = 0.4
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        while not rospy.is_shutdown():
            self.pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        VelocityPublisher()
    except rospy.ROSInterruptException:
        pass
