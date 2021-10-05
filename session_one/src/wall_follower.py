#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

rospy.init_node("wall_follower")

class Bot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.phi = 0

        self.vel = Twist()

        self.threshold = 5
        self.min_dist = 100
        self.min_dist_angle = 90
        self.pub = rospy.Publisher("/cmd_vel", Twist, latch=True, queue_size=10)

        self.target_phi = 0
        self.update_pos()

    def start(self):
        print("Starting")
        self.update_pos()
        self.target_phi = self.phi

        while not self.check_obstacle():
            self.send_vel(0.2,0,0,0.2)
            self.correct_angle()
    
        self.vel.linear.x = 0
        self.pub.publish(self.vel)

        print("Done")

    def update_pos(self):
        sub = rospy.Subscriber('/odom', Odometry, self.update)

    def update(self, a):
        self.x = a.pose.pose.position.x
        self.y = a.pose.pose.position.y

        angles = a.pose.pose.orientation
    
        (roll, yaw, pitch) = euler_from_quaternion([angles.x, angles.y, angles.z, angles.w])

        self.phi = pitch

    
    def callback(self, data):
        self.min_dist = data.ranges[0]

    def check_obstacle(self):
        sub = rospy.Subscriber("/scan", LaserScan, self.callback)

        min_val = self.min_dist

        if min_val >=0.5:
            return False

        else:
            print("Obstacle Found")
            return True
        
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
    
    def correct_angle(self):
        print("Correcting Angle")
        if (self.target_phi - self.phi < 0):
            if (abs(self.target_phi - self.phi)<math.pi):
                while abs(self.target_phi-self.phi) > 0.05:
                    print("1", self.target_phi, self.phi)
                    self.send_vel(0, 0, -0.2, 0.2)

            else:
                while abs(self.target_phi-self.phi) > 0.05:
                    print("2", self.target_phi, self.phi)
                    self.send_vel(0, 0, 0.2, 0.2)

        elif (self.target_phi - self.phi > 0):
            if (abs(self.target_phi - self.phi)<math.pi):
                while abs(self.target_phi-self.phi) > 0.05:
                    print("3", self.target_phi, self.phi)
                    self.send_vel(0, 0, 0.2, 0.2)
                    
            else:
                while abs(self.target_phi-self.phi) > 0.05:
                    print("4", self.target_phi, self.phi)
                    self.send_vel(0, 0, -0.2, 0.2)

        print("Done")


    def free_run(self):
        print("Executing free run")
        self.update_pos()
        self.target_phi = self.phi

        while not self.check_obstacle():
            self.send_vel(0.2,0,0,0.2)
            self.correct_angle()
    
        self.vel.linear.x = 0
        self.pub.publish(self.vel)

        print("Done")

    
    def follow_wall(self):
        print("Following Wall")
        self.update_pos()

        if (self.phi - math.pi/2 > math.pi):
            self.target_phi = -((self.phi - math.pi/2)%(math.pi))

        elif (self.phi - math.pi/2 < -math.pi):
            self.target_phi = (-1*(self.phi - math.pi/2))%(math.pi)

        else:
            self.target_phi = self.phi - math.pi/2

        self.correct_angle()

        print("Done")


if __name__ == "__main__":

    mybot = Bot()

    mybot.start()

    while True:
        mybot.free_run()
        mybot.follow_wall()
            