#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import PID

import time


# class to control drone
class Controller():
    def __init__(self):
        # initalize
        rospy.init_node('controller')

        # inital coordinates for odom so we can use a reference frame from where
        # we took off
        self.first_pos = True
        self.x0 = 0
        self.y0 = 0
        self.z0 = 0

        # keep track of our velocities so that we can log them with odometry
        self.vx = 0
        self.vy = 0
        self.vz = 0

        # for logging data
        self.log_file = open('bebop.csv', 'w')

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)

        # Create the topics we need
        # need to sleep in between them so that they can correctly be created
        rospy.loginfo("Creating Topics")
        self.takeoff_t = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        rospy.sleep(1)
        self.cmd_vel_t = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)
        self.land_t = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        rospy.sleep(1)
        self.record_t = rospy.Publisher('/bebop/record', Bool, queue_size=10)
        rospy.sleep(1)
        self.odom_t = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
        rospy.sleep(5)

        # record video
        rospy.loginfo('Starting recording')
        self.recording(True)

        self.takeoff()

        # set the destination to 3 meters forward, 3 meters right, and 5 meters up
        self.update_goal(3,-3,5)

        # parameters tured for Parrot Bebop 2
        rospy.loginfo('Giving control to PID')
        self.PID_x = PID.PID(SetPoint=self.goal_x, P=.085, D=.35, I=.003)
        self.PID_y = PID.PID(SetPoint=self.goal_y, P=.085, D=.36, I=.006)
        self.PID_z = PID.PID(SetPoint=self.goal_z, P=1)

        # operate at 5 Hz because that is what we get from odometry and the 
        # drone needs commands refreshed otherwise it will just hold position
        rate = rospy.Rate(5)

        # number of cycles of loop
        count = 0

        # move to the destination
        while (1):
            # exit once we have enough footage/earthquake is over
            if count >= 35*5:
                break

            # tell PID of our new odometry position
            self.PID_x.update(self.x)
            self.PID_y.update(self.y)
            self.PID_z.update(self.z)

            # get the velocities from the PID controller
            vx = self.PID_x.output
            vy = self.PID_y.output
            vz = self.PID_z.output

            # store these velocities for logging
            self.vx = vx
            self.vy = vy
            self.vz = vz

            # prepare message to send to drone
            twist = self.empty_twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz

            # send velocities to drone
            self.cmd_vel_t.publish(twist)

            count += 1
            # sleep the loop so we don't busy wait
            rate.sleep()


        rospy.loginfo('Going Home')

        # reset count for return to home
        count = 0

        # 1 meter above the home 
        self.update_goal(0, 0, 1)

        # tolerance for having achieved the destination
        self.tol = 0.1

        # don't want errors from moving to previous goal to influence this one
        self.PID_x.clear_int_term()
        self.PID_y.clear_int_term()
        self.PID_z.clear_int_term()        

        # update the goal
        self.PID_x.SetPoint = self.goal_x
        self.PID_y.SetPoint = self.goal_y
        self.PID_z.SetPoint = self.goal_z


        # now actually move to the goal
        while (1):
            # maximum we allow to move towards goal
            if count >= 20*5:
                break

            # if we are close enough to the goal, we can lang
            if  abs(self.goal_x - self.x) < 0.1 and \
                abs(self.goal_y - self.y) < 0.1 and \
                abs(self.goal_z - self.z) < 0.1:
                rospy.loginfo('Home reached')
                break


            # tell PID of our new odometry position
            self.PID_x.update(self.x)
            self.PID_y.update(self.y)
            self.PID_z.update(self.z)

            # get the velocities from the PID controller
            vx = self.PID_x.output
            vy = self.PID_y.output
            vz = self.PID_z.output

            # store these velocities for logging
            self.vx = vx
            self.vy = vy
            self.vz = vz

            # prepare message to send to drone
            twist = self.empty_twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz

            # send velocities to drone
            self.cmd_vel_t.publish(twist)

            count += 1
            # sleep the loop so we don't busy wait
            rate.sleep()

        # now that we have moved as close to the goal as we are going to, land
        self.land()

        # and stop the recording
        rospy.loginfo('Stopping Recording')
        self.recording(False)

    # update where we want the drone to move to
    def update_goal(self, x=0, y=0, z=0):
        self.log_file.write('0, %5.3f, %5.3f, %5.3f, %5.3f\n'%
            (rospy.get_time(), x, y, z))
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z

    # change the drone's video recording state
    def recording(self, state=True):
        b = Bool()
        b.data = state
        self.record_t.publish(b)

    # make the drone take off
    def takeoff(self):
        rospy.loginfo("Taking off")
        takeoff_msg = Empty()
        self.takeoff_t.publish(takeoff_msg)

        # Let the drone actually take off
        rospy.loginfo("Pausing")
        rospy.sleep(5)

    # make the drone land
    def land(self):
        rospy.loginfo("Landing")
        land_msg = Empty()
        self.land_t.publish(land_msg)

        # make sure the drone actually lands
        rospy.sleep(1)

    # function to be called on program termination for safety
    def shutdown(self):
        rospy.loginfo("Stop Bebop")
        self.cmd_vel_t.publish(Twist())
        self.land()

    # callback called by ROS on availability of new odometry data
    def odom_callback(self, data):
        pos = data.pose.pose.position

        # if this is the first position, update it as such so we have a relative
        # frame
        if (self.first_pos):
            self.x0 = pos.x
            self.y0 = pos.y
            self.z0 = pos.z
            rospy.loginfo('SETTING FRAME ORIGIN TO: %5.3f,%5.3f,%5.3f'%
                (pos.x, pos.y, pos.z))

            self.first_pos = False

        # move to relative frame defined on taking off
        self.x = pos.x - self.x0
        self.y = pos.y - self.y0
        self.z = pos.z - self.z0

        # log position to file
        self.log_file.write('1, %5.3f, %5.3f, %5.3f, %5.3f\n'%
            (rospy.get_time(), self.x, self.y, self.z))

        # log position and velocity to console
        rospy.loginfo('ODOM: %5.3f, %5.3f, %5.3f VEL: %5.3f, %5.3f, %5.3f'%
            (self.x, self.y, self.z, self.vx, self.vy, self.vz))

    # create an empty twist message so that we can update the values which we 
    # want
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
