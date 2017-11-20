#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int32
from add import Functor

rospy.init_node("add")

a = None
b = None

pub = rospy.Publisher("out", Int32, queue_size=1);
f = Functor()

def pubResult(a, b):
    if a and b:
        msg = Int32()
        msg.data = f(a, b)
        pub.publish(msg)

def handleA(msg):
    global a
    a = msg.data
    rospy.loginfo("Got a=%i"%msg.data)
    pubResult(a, b)

def handleB(msg):
    global b
    b = msg.data
    rospy.loginfo("Got b=%i"%msg.data)
    pubResult(a, b)
    

sub = rospy.Subscriber("in0", Int32, handleA);
sub = rospy.Subscriber("in1", Int32, handleB);

rospy.spin()
