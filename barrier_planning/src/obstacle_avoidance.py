#!/usr/bin/env python
"""
Description: This code implementes a finite-time control barrier function
	     to drive the turtlebot from an initial condition to a desired goal region
	     while avoiding the cylindrical obstacle
Author:      Mohit Srinivasan
Date: 	     04/06/2020
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cvxopt import matrix
from cvxopt.blas import dot
from cvxopt.solvers import qp, options
from cvxopt import matrix, sparse

def get_pose(msg):
	
	global x
	global y
	global phi

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	phi = yaw

def go_to_region():
	
	h_goal = -1

	while (h_goal <= 0):
	
		# Barrier function declarations
		gamma = 1	
		x_state = np.array([[x], [y]])


		P_goal = np.array([[1/(0.4)**2, 0], [0, 1/(0.4)**2]])
		P_obstacle1 = np.array([[1/(0.6)**2, 0], [0, 1/(0.6)**2]])
		P_obstacle2 = np.array([[1/(0.6)**2, 0], [0, 1/(0.6)**2]])
		C_goal = np.array([[1], [1]])
		C_obstacle1 = np.array([[0.0], [0.0]])
		C_obstacle2 = np.array([[-1.0], [1.0]])

		h_goal = 1.0 - np.dot(np.dot(np.transpose(x_state - C_goal), P_goal), x_state - C_goal)
		h_obstacle1 = np.dot(np.dot(np.transpose(x_state - C_obstacle1), P_obstacle1), x_state - C_obstacle1) - 1.0
		h_obstacle2 = np.dot(np.dot(np.transpose(x_state - C_obstacle2), P_obstacle2), x_state - C_obstacle2) - 1.0

		# Quadratic program declarations
		A_goal = np.dot(2 * np.transpose(x_state - C_goal), P_goal) 		# Constraint for
		B_goal = gamma * np.sign(h_goal) * (np.absolute(h_goal)**0.4)	        # goal region
		A_goal = np.append(A_goal, 0)		
				
		A_obs1 = np.dot(-2.0 * np.transpose(x_state - C_obstacle1), P_obstacle1) 	# Constraint for
		B_obs1 = gamma * (h_obstacle1)**5						# obstacle
		A_obs1 = np.append(A_obs1, -1)

		A_obs2 = np.dot(-2.0 * np.transpose(x_state - C_obstacle2), P_obstacle2) 	# Constraint for
		B_obs2 = gamma * (h_obstacle2)**5						# obstacle
		A_obs2 = np.append(A_obs2, -1)

		A = np.vstack((A_goal, A_obs1, A_obs2))
		B = np.vstack((B_goal, B_obs1, B_obs2))

		# Quadratic program solver
		H = matrix(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1000]]), tc='d')
		f = matrix(np.array([[0], [0], [0]]), tc='d')
		u_si = qp(H, f, matrix(A), matrix(B))['x']

		# Obtain unicycle velocities using Diffeomorphism technique
		l = 0.3
		R = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
		L = np.array([[1, 0], [0, 1/l]])
		u_turtlebot = np.dot(np.dot(np.transpose(R), L), np.array(u_si)[:2])

		# Publish the velocities to Turtlebot
		vel_msg.linear.x = u_turtlebot[0] * np.cos(phi)
		vel_msg.linear.y = u_turtlebot[0] * np.sin(phi)
		vel_msg.linear.z = 0

		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = u_turtlebot[1]
	
		velocity_publisher.publish(vel_msg)
	
if __name__ == '__main__':
	
	x = 0.0
	y = 0.0
	phi = 0.0

	# ROS initializations
	rospy.init_node('go_to_region', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	odom_sub = rospy.Subscriber('/odom', Odometry, get_pose)
	vel_msg = Twist()

	try:
		go_to_region()

	except rospy.ROSInterruptException:
		pass
