#! /usr/bin/env python

import rospy
from session_one.msg import JangoFett

rospy.init('pub2')
pub = rospy.Publisher('/your_word_is_your_wand', JangoFett, latch=True)
rate = rospy.Rate(2)

out = JangoFett()
out.shape = 'square'
out.side = 10

while not rospy.is_shutdown():
    pub.publish(out)
    rate.sleep()