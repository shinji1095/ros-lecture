#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

def controller():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtlebot_controller', anonymous=False)
    r = rospy.Rate(10) # 10hz
    for _ in range(10):

        velocity = Twist()
        velocity.linear.x = 0.0 # [m/s]
        velocity.linear.y = 0.0 # [m/s]
        velocity.linear.z = 0.0 # [m/s]
        velocity.angular.x = 0.0 # [rad/s]
        velocity.angular.y = 0.0 # [rad/s]
        velocity.angular.z = 0.0 # [rad/s]
        rospy.loginfo( velocity)
        pub.publish( velocity)
        r.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException: pass
