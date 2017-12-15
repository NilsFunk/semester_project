#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TwistStamped

import time


def callback(cmd_velocity):
    move_stamped = TwistStamped()

    move_stamped.twist = cmd_velocity

    now = rospy.get_rostime()
    move_stamped.header.stamp.secs = now.secs
    move_stamped.header.stamp.nsecs = now.nsecs

    move_stampedPub = rospy.Publisher('/hummingbird/copilot/manual_desired_velocity', TwistStamped, queue_size=10)
    move_stampedPub.publish(move_stamped)


def twist_to_twiststamp_node():
    rospy.Subscriber("/cmd_vel", Twist, callback)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('twist_to_twiststamp_node', anonymous=True)
    twist_to_twiststamp_node()