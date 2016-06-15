#!/usr/bin/env python

# Every python controller needs this line
import rospy

#this is for the robot to go back to the base
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

# The velocity command message
from std_msgs.msg import Int32 
from nav_msgs.msg import Odometry

import message_filters
# The laser sensor message
from sensor_msgs.msg import LaserScan

# mathematical helper functions
import math 
import numpy as np
import thread

#this is to pause the robot for some time
import time

import tf 
from tf.transformations import euler_from_quaternion

# This class encapsultes all of the functionality for the Project.


class sendgoal:
    def __init__(self, goalX = 0, goalY = 0, distance=0.5, max_lin_speed= 0.2, max_rot_speed=2,flag =0, counter = 0):
        # Set the stopping distance and max speed
        self.distance = distance
        self.max_lin_speed = max_lin_speed
        self.max_rot_speed = max_rot_speed
	self.flag = flag
	self.counter = counter
 	self.goal_x = goalX
	self.goal_y = goalY
	self.succeeded =True
	self.count = 0
	self.prevGoal_x = 0
	self.prevGoal_y = 0
        # A publisher for the move data
        self.pub = rospy.Publisher('/mobile_base/commands/velocity',
                                   Twist,
                                   queue_size=0)

        # A subscriber for the laser data
        #self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
	#self.sub2 = rospy.Subscriber ('scan',LaserScan,self.callback2, queue_size=1)
    	
        pose_sub = message_filters.Subscriber('odom',Odometry, queue_size=1)
        scan_sub = message_filters.Subscriber('scan',LaserScan, queue_size=1)
        ts = message_filters.ApproximateTimeSynchronizer([pose_sub, scan_sub], 0,1)
        ts.registerCallback(self.callback)

		


  	pose_sub1 = message_filters.Subscriber('odom',Odometry, queue_size=1)
        scan_sub1 = message_filters.Subscriber('scan',LaserScan, queue_size=1)
	ts1 = message_filters.ApproximateTimeSynchronizer([pose_sub1, scan_sub1], 1, 1)
	ts1.registerCallback(self.generateGoal)

    def callback2(self,pose_msg):
	#print 'pose', pose_msg.ranges[0]
	pass
	
			
#This function is for the Robot to go back to base
#This is called whenever the items on the robots are over
#This can be changed by setting the self.counter to different values. 
    def GoToBase(self):
	
	command1 = Twist()
	rospy.on_shutdown(self.shutdown)
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    	rospy.loginfo("wait for the action server to come up: ")
	self.move_base.wait_for_server(rospy.Duration(5))
	goal = MoveBaseGoal()
   	goal.target_pose.header.frame_id = 'base_link'
    	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose = Pose(Point(self.goal_x,self.goal_y , 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
	self.move_base.send_goal(goal)
	success = self.move_base.wait_for_result(rospy.Duration(20)) 

    	if not success:
                self.move_base.cancel_goal()
                rospy.loginfo("The base failed to reach the desired pose")
    	else:
        # We made it!
		state = self.move_base.get_state()
        	if state == GoalStatus.SUCCEEDED:
            	    rospy.loginfo("Hooray, reached the desired pose")
		    #time.sleep(1)
		    #command1.linear.x = 0.0
		
    def Serve(self,scan_msg, pose_msg):
	pass
      
		    



		    
    def generateGoal(self, pose_data, scan_data):
	print 'hu', scan_data.ranges[0]
	#-----------------------------Find Robots Orientation---------------------------
	quaternion = (pose_data.pose.pose.orientation.x,
			pose_data.pose.pose.orientation.y, 
			pose_data.pose.pose.orientation.z, 
			pose_data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	#-------------------------------------------------------------------------------
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	#--------------------Correct Orientation to be between 0 to 2pi-----------------
	if yaw < 0:
		yaw = yaw + 2*math.pi
	#-------------------------------------------------------------------------------
	delta = yaw - 0.506145477295 #angle of first array
	maxIndice = np.argmax(scan_data.ranges)
	theta = maxIndice*0.00158417993225 + delta
	
	#self.goal_x = self.goal_x+scan_data.ranges[maxIndice]*math.cos(theta)
	#self.goal_y = self.goal_y+scan_data.ranges[maxIndice]*math.sin(theta)
	if scan_data.ranges[maxIndice] > 0.3:
		print 'yes'
		self.goal_x = 0.3*math.cos(theta)
		self.goal_y = 0.3*math.sin(theta)
		#self.goal_x = pose_data.pose.pose.position.x+3.0*math.cos(theta)
		#self.goal_y = pose_data.pose.pose.position.y+3.0*math.sin(theta)
	else:
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.1
		
	print maxIndice, theta, self.goal_x, self.goal_y

	

    def shutdown(self):
        rospy.loginfo("Stop")
	
    def input_thread(self,L):
	    raw_input()
	    L.append(None)
    
    def callback(self, pose_msg2, scan_msg2):
	
	self.xx=pose_msg2.pose.pose.position.x
	self.yy=pose_msg2.pose.pose.position.y

	if pose_msg2.header.stamp.secs == 5:
            self.robot_x = pose_msg2.pose.pose.position.x
            self.robot_y = pose_msg2.pose.pose.position.y
		    

	self.Serve(scan_msg2, pose_msg2)
	
	



if __name__ == '__main__':
    

    rospy.init_node('sendgoal')
    stopper = sendgoal()

    rospy.spin()
