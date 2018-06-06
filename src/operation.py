#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import time


def takeoff(takeoff_t):
    print "Taking off"
    takeoff_msg = Empty()
    takeoff_t.publish(takeoff_msg)
    rospy.sleep(1)

def land(land_t):
    print "Landing"
    land_msg = Empty()
    land_t.publish(land_msg)
    rospy.sleep(1)
    
def empty_twist():
    twist = Twist()

    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0

    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    return twist

def controller():

    rospy.init_node('simple_test')

    print "creating topics"
    takeoff_t = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
    rospy.sleep(1)
    cmd_vel_t = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    land_t = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    rospy.sleep(1)
    
    rospy.sleep(5)
    
    takeoff(takeoff_t)

    # Let the drone actually take off
    print "Pausing"
    rospy.sleep(5)

    # need to refresh the command on the periodic interval
    rate = rospy.Rate(10) # 10 Hz
    t0 = time.time()

    
    print "Moving up"
    while (time.time() - t0 < 2):
        twist = empty_twist()
        twist.linear.z = 1
        
        cmd_vel_t.publish(twist)
        rate.sleep()



    rospy.sleep(2)
    land(land_t)

    
if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
