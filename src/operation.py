#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped

def takeoff():
    takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
    rospy.sleep(1)
    takeoff_msg = Empty()
    takeoff.publish(takeoff_msg)

def land():
    land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    rospy.sleep(1)
    land_msg = Empty()
    land.publish(land_msg)
    

def talker():

    rospy.init_node('simple_test')
    
    takeoff()
    rospy.sleep(5)
    land()

    
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     goal = PoseStamped()
    #     goal.header.frame_id = "/map"
    #     goal.header.stamp = rospy.Time.now()
    #     goal.pose.position.x = 3.0
    #     goal.pose.position.y = 1.0
    #     goal.pose.position.z = 0.0
    #     goal.pose.orientation.w = 1.0
    #     pub.publish(goal)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
