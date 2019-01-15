#!/usr/bin/env python

""" nav_test.py - Version 0.1 2012-01-10

    Command a robot to move autonomously among a number of goal locations defined in the map frame.
    On each round, select a new random sequence of locations, then attempt to move to each location
    in succession.  Keep track of success rate, time elapsed, and total distance traveled.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""
import roslib; #roslib.load_manifest('rbx1_nav')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

stop=0

class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

def Stop(data) :
	if data.data=='1' :
		rospy.loginfo('stop')
		global stop
		stop=1
		print("robobo stop")
		rospy.signal_shutdown('Quit navig')


def Move(data):
	if data.data=="1":
	
		rospy.loginfo("mouvement")
		print(data.data)
		goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
		           'SUCCEEDED', 'ABORTED', 'REJECTED',
		           'PREEMPTING', 'RECALLING', 'RECALLED',
		           'LOST']

		locations = dict()
		#On donne ici la position du noeud ou l'evenement s'est produit
		#locations['i5_droite']=Pose(Point(1,13,0.000),Quaternion(0.000,0.000,0.892,-0.451))
		locations['goal'] = Pose(Point(-1.82, 11.3, 0.00641), Quaternion(0.000, 0.000, 0.892, -0.451))
		sequence = ['goal']


		rospy.loginfo("sequence "+str(sequence))
		location = sequence[0]
		goal = MoveBaseGoal()
		goal.target_pose.pose = locations[location]
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		rospy.loginfo("Going to: " + str(location))

		move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		move_base.send_goal(goal)
		finished_within_time = move_base.wait_for_result(rospy.Duration(300))
		 # Check for success or failure
		if not finished_within_time:
		    move_base.cancel_goal()
		    rospy.loginfo("Timed out achieving goal")
		else:
		    state = move_base.get_state()

		    rospy.sleep(1)
		    #if(str(location)=='goal'):
		    	#state = self.move_base.get_state()
			#print('goal')
		    if state == GoalStatus.SUCCEEDED:
		        if(str(location) == 'goal'):
                        	camera = TakePhoto()
                        	img_title = rospy.get_param('~image_title', 'photo.jpg')
                        	if camera.take_picture(img_title):
                                	rospy.loginfo("Saved image " + img_title)
                        	else:
                                	rospy.loginfo("No images received")

			rospy.loginfo("Goal succeeded!")
		        rospy.loginfo("State:" + str(state))
		    else:
		      rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
	

class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        locations = dict()
        
	locations['couloir_milieu'] = Pose(Point(-1.35, 9.31, 0.000), Quaternion(0.000, 0.000, -0.670, 0.743))
 	locations['i5_droite'] = Pose(Point(1, 13, 0.000), Quaternion(0.000, 0.000, 0.892, -0.451))
        locations['i5_gauche'] = Pose(Point(1.13, 6.94, 0.000), Quaternion(0.000, 0.000, 0.480, 0.877))
	locations['goal'] = Pose(Point(-1.82, 11.3, 0.00641), Quaternion(0.000, 0.000, 0.892, -0.451))

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 10 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(10))

        # subscribe au topic chatter, avec un appel a la fonction de callback
        rospy.Subscriber("/chatter",String, Move)
	rospy.Subscriber("/arret",String, Stop)
        
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = 3 
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        

        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown() :
            rospy.loginfo("*** MODE SEQUENTIEL")
            pub_mode = rospy.Publisher("activity_mode", String, queue_size=15)
            pub_mode.publish("SEQUENTIEL")

            # If we've gone through the current sequence,
            # start with a new random sequence
            if i == n_locations:
                i = 0
                #sequence = [sample(locations, n_locations)]

                # On remplace l'aleatoire des positions par un ordre de position bien defini
		sequence = ["i5_droite","couloir_milieu","i5_gauche"]
                rospy.loginfo("sequence "+str(sequence))
                # Skip over first location if it is the same as
                # the last location
                #if sequence[0] == last_location:
                #    i = 1
            
            # Get the next location in the current sequence
            location = sequence[i]
                        
            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x - 
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y - 
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x - 
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y - 
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""
            
            # Store the last location for distance calculations
            last_location = location
            
            # Increment the counters
            i += 1
            n_goals += 1
        
            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
                


            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            
            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            rospy.sleep(self.rest_time)
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
