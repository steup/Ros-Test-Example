#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int32

rospy.init_node("test_sub")

def handleInput(i):
    rospy.loginfo("Current result %i"%i.data)

sub = rospy.Subscriber("in", Int32, handleInput);

rospy.spin()
