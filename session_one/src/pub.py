#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32

rospy.init_node('pub')

pub = rospy.Publisher('/your_word_is_your_wand', Int32, latch = True, queue_size=10)
rate = rospy.Rate(2)

while not rospy.is_shutdown():
    pub.publish(10)
    rate.sleep()