#!/usr/bin/env python
import rospy
from math import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def scan_recieved(msg):
    # processes data from laser scan, msg is of type sensor_msgs/LaserScan
    valid_ranges = {} # dictionary where key=distance and value=angle from 0 (0-360)
    sum_x = 0
    sum_y = 0
    for i in range(360):
        if msg.ranges[i] > 0.1 and msg.ranges[i]<2:
            valid_ranges[msg.ranges[i]] = i
    for distance in valid_ranges:
        mag = 0.1/(distance) # as distance gets bigger, magnitude of force decreases
        rad = radians(valid_ranges[distance])
        sum_x += mag*sin(rad) # sum of forces in the x direction
        sum_y += mag*(-1)*cos(rad) # sum of forces in the y direction, front of the robot as positive
    sum_y += 2 #tendency to go straight
    print ("sumx ") + str(sum_x)
    print ("sumy ") + str(sum_y)
    angle = atan(sum_y/sum_x)
    magnitude = sqrt(sum_x**2 + sum_y**2)
    print "angle " + str(angle)
    print "magnitude "+ str(magnitude)
    publish_twist_velocity(.03*magnitude,0,0,0,0,2*angle)    

def publish_twist_velocity (x,y,z,xa,ya,za):
    pub = rospy.Publisher ('cmd_vel', Twist, queue_size=10) #name of topic, typer of topic ("rostopic type /cmd_vel")
    msg = Twist (Vector3 (x, y, z), Vector3 (xa, ya, za))
    pub.publish (msg)

def teleop ():
    #publisher writes  - here, to cmd_vel
    pub = rospy.Publisher ('cmd_vel', Twist, queue_size=10) #name of topic, typer of topic ("rostopic type /cmd_vel")

    #subscriber reads - here, from scan
    sub = rospy.Subscriber ('scan', LaserScan, scan_recieved) #name, type, callback - could add another arg like pub so we can publish

    rospy.init_node ('teleop', anonymous=True)
    r = rospy.Rate (10) # 10hz
    while not rospy.is_shutdown ():
        r.sleep ()
        
if __name__ == '__main__':
    try:
        teleop ()
    except rospy.ROSInterruptException: pass
