#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf2_geometry_msgs

from tf.transformations import *
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams


ball_estimated_x_pos = 0
ball_estimated_y_pos = 0
ball_estimated_z_pos = 0

initial_tool_x_pos = 0
initial_tool_y_pos = 0
initial_tool_z_pos = 0
roll = 0
pitch = 0
yaw = 0


# modify this to just add it to the plan, don't return it.
def create_new_pt(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, plan):
	"""
	Adds new Twist message with given params to the given plan.	
	"""
	pt = Twist()
	pt.linear.x = linear_x
	pt.linear.y = linear_y
	pt.linear.z = linear_z
	pt.angular.x = angular_x
	pt.angular.y = angular_y
	pt.angular.z = angular_z
	
	plan.points.append(pt)


def sphere_params_callback(params):
	#print(f'xc: {params.xc}, yc: {params.yc}, zc: {params.zc}, radius: {params.radius}')
	global ball_estimated_x_pos
	global ball_estimated_y_pos
	global ball_estimated_z_pos
	
	ball_estimated_x_pos = params.xc
	ball_estimated_y_pos = params.yc
	ball_estimated_z_pos = params.zc


def tool_position_callback(position):
	global initial_tool_x_pos
	global initial_tool_y_pos
	global initial_tool_z_pos
	global roll
	global pitch
	global yaw
	
	initial_tool_x_pos = position.linear.x
	initial_tool_y_pos = position.linear.y
	initial_tool_z_pos = position.linear.z
	roll = position.angular.x
	pitch = position.angular.y
	yaw = position.angular.z


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	# subscribe to sphere_params, get ball pos
	rospy.Subscriber('/sphere_params', SphereParams, sphere_params_callback)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# add stuff for the ROS transform listener:
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# add a subscriber to get the robot's initial position
	rospy.Subscriber('/ur5e/toolpose', Twist, tool_position_callback)
	# try slowing this down a bit, see if this fixes our issues...
	loop_rate = rospy.Rate(3)

	# define a plan variable
	plan = Plan()
	plan_generated = False
	
	# do everything in relation to the base frame?
	while not rospy.is_shutdown():
		# try to get updated transform between base and camera
		try:
			trans = tfBuffer.lookup_transform('base', 'camera_color_optical_frame', rospy.Time())
			
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('issue getting frames')
			loop_rate.sleep()
			continue
		
		# now declare a point that represents the ball's position in the camera frame
		ball_in_camera_frame = tf2_geometry_msgs.PointStamped()
		ball_in_camera_frame.header.frame_id = 'camera_color_optical_frame'
		ball_in_camera_frame.header.stamp = rospy.get_rostime()
		# use the coordinates obtained from the sphere fit node:
		ball_in_camera_frame.point.x = ball_estimated_x_pos
		ball_in_camera_frame.point.y = ball_estimated_y_pos
		ball_in_camera_frame.point.z = ball_estimated_z_pos

		
		# now convert the ball to the base frame:
		ball_in_base_frame = tfBuffer.transform(ball_in_camera_frame, 'base', rospy.Duration(1.0))
		
		# add the start position obtained from the /ur5e/toolpose subscriber to the plan
		create_new_pt(initial_tool_x_pos, initial_tool_y_pos, initial_tool_z_pos, roll, pitch, yaw, plan)
		
		if not plan_generated:
			# above ball point
			create_new_pt(ball_in_base_frame.point.x, ball_in_base_frame.point.y, ball_in_base_frame.point.z + 0.1, roll, pitch, yaw, plan)
			# center of the ball, 2cm offset to account for vision node imperfections
			create_new_pt(ball_in_base_frame.point.x, ball_in_base_frame.point.y, ball_in_base_frame.point.z+0.02, roll, pitch, yaw, plan)
			# above drop point
			create_new_pt(ball_in_base_frame.point.x + 0.3, ball_in_base_frame.point.y + 0.1, ball_in_base_frame.point.z+0.2, roll, pitch, yaw, plan)
			# the drop point
			create_new_pt(ball_in_base_frame.point.x + 0.3, ball_in_base_frame.point.y + 0.1, ball_in_base_frame.point.z+0.1, roll, pitch, yaw, plan)
			
			plan_generated = True
			
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
