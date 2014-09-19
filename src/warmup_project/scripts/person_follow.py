#!/usr/bin/env python
import rospy
from math import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class PetRobot:

    def __init__(self):
        rospy.init_node('pet_robot', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)
        self.state = 1 # 0 is no objects detected, spin mode. 1 is object detected, follow mode

    def scan_received(self, msg):
        # processes data from laser scan, msg is of type sensor_msgs/LaserScan
        self.state = 0
        distance = 0
        angle = 3.14
        func = 0
        intensity = 0
        for i in range(360):
            if msg.ranges[i] > .21 and msg.ranges[i] < 3: # only look at objects in "donut" of view
                self.state = 1 # state control - found an object so we're in follow state
                angle_rads = radians(i)
                if (angle_rads > 3.14): # convert to positives and negatives
                    angle_rads = angle_rads - 6.28
                print "i " + str(msg.intensities[i]/15.0)
                print "d " + str(1.5/(msg.ranges[i]))
                print "a " + str(3*cos(.5*angle_rads))
                #if msg.ranges[i] < distance: # smallest distance (closest to robot) wins
                #if msg.intensities[i] > intensity: #brightest, most reflective object (white) wins
                if self.optimize(msg.ranges[i], angle_rads, msg.intensities[i]) > func:
                    intensity = msg.intensities[i]
                    distance = msg.ranges[i]
                    angle = angle_rads
        error = distance - .75 # about 2.5 ft away from person of choice, almost right at the Werner Average for personal space
        velocity = 0.3*error # proportional control
        angle = .9*angle # proportional control (error = angle - 0, because 0 is front)
        if abs(angle) < 0.05 and abs(velocity) < 0.05: # if it's doing tiny adjustments just stay still
            angle = 0
            velocity = 0
        if not self.state: # found no object so let's spin
            angle = 0
            velocity = 1
        self.publish_twist_velocity(velocity, angle)

    # a guesstimated/guess-n-checked equation that takes into account distance, angle, and intensity of object
    # to try to determine if it's worth following
    def optimize(self,d,a,i):
        return (i/15.0) + 3*cos(.5*a) + 1.5/(d)

    # publish velocity to robot
    def publish_twist_velocity (self,x,a):
        msg = Twist (Vector3 (x, 0, 0), Vector3 (0, 0, a))
        self.pub.publish (msg)

    def run(self):
        r = rospy.Rate (10) # 10hz
        while not rospy.is_shutdown ():
            r.sleep ()
        
if __name__ == '__main__':
    try:
        node = PetRobot()
        node.run()
    except rospy.ROSInterruptException: pass
