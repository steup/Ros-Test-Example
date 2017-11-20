#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int32
import time

rospy.init_node("test_pub", anonymous=True)

pub = rospy.Publisher("out", Int32, queue_size=1);

msg=Int32()
msg.data=0

while True:
   msg.data=msg.data+1
   pub.publish(msg)
   rospy.loginfo("Published %i"%msg.data)
   time.sleep(1)
