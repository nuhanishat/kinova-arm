#! /usr/bin/env python

# Author: Nuha Nishat
# Date: 1/30/20

import rospy
import sys, os
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
import tf, math
import tf.transformations
import pdb
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi 
from std_msgs.msg import String

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, AllowedCollisionMatrix
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
import time
import csv
import copy



# move_group_python_interface_tutorial was used as reference

class MoveRobot():
	def __init__(self):
		# Initialize moveit commander and ros node for moveit
		moveit_commander.roscpp_initialize(sys.argv)

		# Initializing node
		rospy.init_node("move_kinova", anonymous=True)

		# Define robot using RobotCommander. Provided robot info such as
		# kinematic model and current joint state
		self.robot = moveit_commander.RobotCommander()

		# Setting the world 
		self.scene = moveit_commander.PlanningSceneInterface()

		# Define the planning group for the arm you are using
		# You can easily look it up on rviz under the MotionPlanning tab
		self.move_group = moveit_commander.MoveGroupCommander("arm")
		self.move_gripper = moveit_commander.MoveGroupCommander("gripper")
		
		# Set the precision of the robot
		rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
		
		rospy.wait_for_service("/apply_planning_scene", 10.0)
		rospy.wait_for_service("/get_planning_scene", 10.0)

		self.apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
		self.get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
		rospy.sleep(2)

		self.disp_pub = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
	
		# To see the trajectory
		self.disp = moveit_msgs.msg.DisplayTrajectory()

		self.disp.trajectory_start = self.robot.get_current_state()
		
		self.rate = rospy.Rate(10)

		self.move_group.allow_replanning(1)

		self.main()
	
	def set_planner_type(self, planner_name):
		if planner_name == "RRT":
			self.move_group.set_planner_id("RRTConnectkConfigDefault")
		if planner_name == "RRT*":
			self.move_group.set_planner_id("RRTstarkConfigDefault")
		if planner_name == "PRM*":
			self.move_group.set_planner_id("PRMstarkConfigDefault")


	def go_to_joint_state(self, joint_state):
		joint_goal = JointState()
		joint_goal.position = joint_state
		self.move_group.set_joint_value_target(joint_goal.position)

		self.plan = self.move_group.plan()
		self.move_group.go(wait=True)
		self.move_group.execute(self.plan, wait=True)

		self.move_group.stop()
		self.move_group.clear_pose_targets()
		rospy.sleep(1)

	def go_to_goal(self, ee_pose):
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = ee_pose[0]
		pose_goal.position.y = ee_pose[1]
		pose_goal.position.z = ee_pose[2]
		
		if len(ee_pose) == 6:
			quat = tf.transformations.quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]), math.radians(ee_pose[5]))
			pose_goal.orientation.x = quat[0]
			pose_goal.orientation.y = quat[1]
			pose_goal.orientation.z = quat[2]
			pose_goal.orientation.w = quat[3]

		else:
			pose_goal.orientation.x = ee_pose[3]
			pose_goal.orientation.y = ee_pose[4]
			pose_goal.orientation.z = ee_pose[5]
			pose_goal.orientation.w = ee_pose[6]	

		self.move_group.set_pose_target(pose_goal)
		self.move_group.set_planning_time(20)
		# rospy.sleep(1)
		self.move_group.go(wait=True)
		self.move_group.stop()

		self.move_group.clear_pose_targets()
		rospy.sleep(1)

	def move_gripper(self, cmd):
		if cmd == "Close":
			self.move_gripper.set_named_target("Close")
		elif cmd == "Open":
			self.move_gripper.set_named_target("Open")
		else: 
			self.move_gripper.set_joint_value_target(cmd)
		self.move_gripper.go(wait=True)
		rospy.sleep(2)

	def display_trajectory(self):
		self.disp_pub = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		self.disp.trajectory.append(self.plan)
		print(self.disp.trajectory)
		self.disp_pub.publish(self.disp)

	def read_joint_angles(self, file_name):
		self.joints = []
		with open(file_name) as csv_file:
			angles = []
			reader = csv.reader(csv_file, delimiter=',')
			for row in reader:
				angles = [float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6])]
				self.joints.append(angles)

	# Function that reads in the waypoints from txt file provided by the waypoints generator
	def get_delta_path(self, file_name):
		self.waypoints = []
		self.change = []
		with open(file_name) as csv_file:
			point = []
			reader = csv.reader(csv_file, delimiter=',')
			for row in reader:
				point = [float(row[0]), float(row[1]), float(row[2])]
				self.waypoints.append(point)

		for i in range(len(self.waypoints)-1):
			delta = []
			for j in range(len(self.waypoints[i])):
				delta.append(float(format(self.waypoints[i+1][j] - self.waypoints[i][j])))

			self.change.append(delta)

		print(self.change)

		

	def get_adjusted_waypoints(self, file_name, starting_pose):
		self.get_delta_path(file_name)
		# The cumulutive change from starting position to generate 
		self.c_change = []

		start = self.waypoints[0]

		if len(starting_pose) == 6:
			quat = tf.transformations.quaternion_from_euler(math.radians(starting_pose[3]), math.radians(starting_pose[4]), math.radians(starting_pose[5]))
			starting_pose[3] = quat[0]
			starting_pose[4] = quat[1]
			starting_pose[5] = quat[2]
			starting_pose.append(float(format(quat[3], '.2f')))

		for i in range(1, len(self.waypoints)):
			self.c_change.append([float(format(self.waypoints[i][0] - start[0] + starting_pose[0], '.2f')), float(format(self.waypoints[i][1] - start[1] + starting_pose[1], '.2f')), float(format(self.waypoints[i][2] - start[2] + starting_pose[2], '.2f')), starting_pose[3], starting_pose[4], starting_pose[5], starting_pose[6]])

		print(self.c_change)


	def execute_path(self):
		rospy.loginfo('Waiting while plan is displayed')
		self.move_group.go(wait=True)
		self.move_group.execute(self.plan, wait=True)

	# Computes cartesian Trajectory
	def test_trajectory(self, starting_pose):
		
		waypoints = []

		# start with the current pose
		# waypoints.append(self.move_group.get_current_pose().pose)

		waypoints.append(Pose(Point(starting_pose[0], starting_pose[1], starting_pose[2]), Quaternion(self.c_change[0][3], self.c_change[0][4], self.c_change[0][5], self.c_change[0][6])))

		# first oreint gripper and move forward (+x)
		for i in range(len(self.c_change)):
			point = self.c_change[i]
			waypoints.append(Pose(Point(point[0], point[1], point[2]),Quaternion(point[3], point[4], point[5], point[6])))
		
		# print(waypoints[0:3])

		(self.plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.02, 0.0)
		
		rospy.sleep(2.0)
		
		self.move_group.go(wait=True)
		self.move_group.execute(self.plan, wait=True)

		
		rospy.sleep(20.0)


		self.move_group.clear_pose_targets()

	def main(self):

		# Set up path here

		# Pick planner 
		self.set_planner_type("RRT")

		# while not rospy.is_shutdown():
		# 	# Draw a straight line in 90 deg

		# rospy.loginfo('Going to first point')
		# self.go_to_goal([-0.1, -0.63, 0.2, 0, 180, 0])

		start = [-0.1, -0.6, 0.2, 0, 180, 0]
		# start1 = [-0.2, -0.6, 0.2, 0, 135, 0]

		rospy.loginfo('Trying trajectory planning')
		# self.get_delta_path('list1.txt')
		self.get_adjusted_waypoints('list1.txt', start)

		
		# while not rospy.is_shutdown():
		rospy.loginfo("Going to starting point")
		self.go_to_goal(start)
		rospy.sleep(3.0)
		rospy.loginfo('Planning Path')
		rospy.loginfo('Displaying Trajectory')
		self.test_trajectory(start)


		rospy.spin()


		# self.go_to_goal(start1)

		# for i in range(len(self.c_change)):
		# 	rospy.loginfo('Point ' + str(i+1))
		# 	rospy.loginfo(self.c_change[i])
		# 	self.go_to_goal(self.c_change[i])




		# 	rospy.loginfo('Moving down')
		# 	self.go_to_goal([-0.1, -0.63, 0.097, 0, 180, 0])
			
		# 	rospy.loginfo("Going to second point")
		# 	self.go_to_goal([0.1, -0.63, 0.097, 0, 180, 0])

		# 	rospy.loginfo('Moving Up')
		# 	self.go_to_goal([0.1, -0.63, 0.2, 0, 180, 0])


		# self.read_joint_angles('joints_list_new.txt')
		# print(self.joints)

		# for i in range(len(self.joints)):
		# 	rospy.loginfo('Joint Config ' + str(i))
		# 	self.go_to_joint_state(self.joints[i])

		




if __name__ == '__main__':
	MoveRobot()
	


