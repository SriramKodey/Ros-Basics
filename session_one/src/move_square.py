#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

def curr_pos(pos):
    x = pos.pose.pose.position.x
    y = pos.pose.pose.position.y
    
    angles = pos.pose.pose.orientation
    
    (roll, yaw, pitch) = euler_from_quaternion([angles.x, angles.y, angles.z, angles.w])

    phi = pitch
    print(x, y, phi)

def move(tps):
    send = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, latch = True, queue_size=10)

    for i in range(4):
        rospy.Subscriber('/odom', Odometry, curr_pos)
        send.linear.x = 0.2
        pub.publish(send)
        rospy.sleep(tps)

        send.linear.x = 0
        send.angular.z = (math.pi/10)

        pub.publish(send)
        rospy.sleep(5)
        
        send.angular.z = 0
        rospy.sleep(1)
        rospy.Subscriber('/odom', Odometry, curr_pos)



def callback_function(a):
    side = a.data
    move(side)
    

rospy.init_node('move_square')
rospy.Subscriber('/your_word_is_your_wand', Int32, callback_function)
rospy.spin()
