#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import PID

import time

"""
TODO
    parameter tuning for PID
    add differentiation to PID
    log odom and velocity to file
    maybe turn off PID controller when we are in vicinity of goal
"""

class Controller():
    def __init__(self):
        # initalize
        rospy.init_node('controller')

        # inital coordinates for odom
        self.first_pos = True
        self.x0 = 0
        self.y0 = 0
        self.z0 = 0

        # for logging data
        self.log_file = open('bebop.csv', 'w')

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

        self.update_goal(0, 0, 2)

        rospy.loginfo('Giving control to PID')
        self.PID_x = PID.PID(SetPoint=self.goal_x, P=.4)
        self.PID_y = PID.PID(SetPoint=self.goal_y, P=.4)
        self.PID_z = PID.PID(SetPoint=self.goal_z, P=1)

        rate = rospy.Rate(5)

        count = 0

        while (1):

            self.PID_x.update(self.x)
            self.PID_y.update(self.y)
            self.PID_z.update(self.z)

            vx = self.PID_x.output
            vy = self.PID_y.output
            vz = self.PID_z.output

            rospy.loginfo('%5.3f %5.3f %5.3f'%(vx,vy,vz))

            twist = self.empty_twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz

            self.cmd_vel_t.publish(twist)

            count += 1
            rate.sleep()

        self.land()

    def update_goal(self, x=0, y=0, z=0):
        self.log_file.write('0, %5.3f, %5.3f, %5.3f, %5.3f'%(rospy.get_time(), x, y, y))
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z

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
        self.cmd_vel_t.publish(Twist())
        self.land()
        # rospy.sleep(1)

    def odom_callback(self, data):
        pos = data.pose.pose.position
        ve  = data.twist.twist.linear

        self.x = pos.x - self.x0
        self.y = pos.y - self.y0
        self.z = pos.z - self.z0

        if (self.first_pos):
            self.x0 = self.x
            self.y0 = self.y
            self.z0 = self.z
            rospy.loginfo('SETTING FRAME ORIGIN TO: %5.3f,%5.3f,%5.3f'%(self.x, self.y, self.z))
            self.first_pos = False

        self.log_file.write('1, %5.3f, %5.3f, %5.3f, %5.3f'%(rospy.get_time(), self.x, self.y, self.z))
        rospy.loginfo('ODOM: %5.3f,%5.3f,%5.3f'%(self.x, self.y, self.z))
        # rospy.loginfo('%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f'%(rospy.get_time(),pose.position.x, pose.position.y, pose.position.z, vel.x, vel.y, vel.z))

    def empty_twist(self):
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
