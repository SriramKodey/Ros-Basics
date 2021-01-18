#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

rospy.init_node('move_improved')

def get_distance(a, b, x, y):
    distance = math.sqrt((pow(a-x, 2) + pow(b-y, 2)))
    return distance


class goal:
    '''Input x and y'''
    def __init__(self, a, b):
        self.x = a
        self.y = b

    

class bot:
    ''' Input x, y, and phi (initial values) '''
    vel = Twist()
    def __init__(self, a, b, c):
        self.x = a
        self.y = b
        self.phi = c

    def update_pos(self, a):
        self.x = a.pose.pose.position.x
        self.y = a.pose.pose.position.y

        angles = a.pose.pose.orientation
    
        (roll, yaw, pitch) = euler_from_quaternion([angles.x, angles.y, angles.z, angles.w])

        self.phi = pitch
        

    def send_vel(self, a, b, c, time):
        pub = rospy.Publisher('/cmd_vel', Twist, latch=True, queue_size=10)
        self.vel.linear.x = a
        self.vel.linear.y = b
        self.vel.angular.z = c
        pub.publish(self.vel)
        rospy.sleep(time)

        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.angular.z = 0
        pub.publish(self.vel)


def go_to_goal(mygoal, mybot):
    dist = 10
    while dist >= 0.1:
        rospy.Subscriber('/odom', Odometry, mybot.update_pos)
        dist = get_distance(mygoal.x, mygoal.y, mybot.x, mybot.y)
        mybot.send_vel(0,0,0.1,0.1)

        vec = [mygoal.x - mybot.x , mygoal.y - mybot.y]

        phi = math.atan(vec[1]/vec[0])
        
        if vec[0] < 0 :
            if vec[1] >= 0:
                phi = math.pi + phi
            
            elif vec[1] < 0:
                phi = phi - math.pi

        if (phi - mybot.phi < 0):
            if (abs(phi - mybot.phi)<math.pi/2):
                while abs(phi-mybot.phi) > 0.05:
                    mybot.send_vel(0, 0, -0.1, 0.2)
                    print(phi, mybot.phi)

            else:
                while abs(phi-mybot.phi) > 0.05:
                    mybot.send_vel(0, 0, 0.1, 0.2)
                    print(phi, mybot.phi)

        elif (phi - mybot.phi > 0):
            if (abs(phi - mybot.phi)<math.pi/2):
                while abs(phi-mybot.phi) > 0.05:
                    mybot.send_vel(0, 0, 0.1, 0.2)
                    print(phi, mybot.phi)
            else:
                while abs(phi-mybot.phi) > 0.05:
                    mybot.send_vel(0, 0, -0.1, 0.2)
                    print(phi, mybot.phi)

        
        mybot.send_vel(0.2, 0, 0, dist)



def main():
    mygoal = goal(0, 5)
    mybot = bot(0, 0, 0)
    go_to_goal(mygoal, mybot)



if __name__ == "__main__":
    main()


    


    