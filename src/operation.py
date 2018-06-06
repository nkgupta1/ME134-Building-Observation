#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


import time

class Controller():
    def __init__(self):
        # initalize
        rospy.init_node('controller')

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)

        # Create the topics we need
        rospy.loginfo("Creating Topics")
        self.takeoff_t = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        rospy.sleep(1)
        self.cmd_vel_t = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)
        self.land_t = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        rospy.sleep(1)
        self.odom_t = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
        rospy.sleep(5)

        self.takeoff()

        move(direct=(0,0,1), t=1)

        land()
        

    def move(direct=(0,0,0), t=0):
        # need to refresh the command on the periodic interval
        rate = rospy.Rate(10) # 10 Hz
        t0 = time.time()

        
        rospy.loginfo('Moving')
        while (time.time() - t0 < t):
            twist = empty_twist()
            twist.linear.x = direct[0]
            twist.linear.y = direct[1]
            twist.linear.z = direct[2]
            
            cmd_vel_t.publish(twist)
            rate.sleep()

        twist = empty_twist()
        self.cmd_vel_t.publish(twist)

    def takeoff(self):
        rospy.loginfo("Taking off")
        takeoff_msg = Empty()
        self.takeoff_t.publish(takeoff_msg)

        # Let the drone actually take off
        rospy.loginfo("Pausing")
        rospy.sleep(5)

    def land(self):
        rospy.loginfo("Landing")
        land_msg = Empty()
        self.land_t.publish(land_msg)
        rospy.sleep(1)


    def shutdown(self):
        rospy.loginfo("Stop Bebop")
        self.cmd_vel.publish(Twist())
        self.land()
        rospy.sleep(1)

    def odom_callback(self, data):
        pose = data.pose.pose
        vel  = data.twist.twist.linear

        rospy.loginfo('%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f'%(rospy.get_time(),pose.position.x, pose.position.y, pose.position.z, vel.x, vel.y, vel.z))

    def empty_twist():
        twist = Twist()

        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        return twist
    

    
if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Failed")
