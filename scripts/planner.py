#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf2_geometry_msgs

from tf.transformations import *
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams


estimated_x_pos = 0
estimated_y_pos = 0
estimated_z_pos = 0


def create_new_pt(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
	"""
	Returns a new Twist message with given params.	
	"""
	pt = Twist()
	pt.linear.x = linear_x
	pt.linear.y = linear_y
	pt.linear.z = linear_z
	pt.angular.x = angular_x
	pt.angular.y = angular_y
	pt.angular.z = angular_z
	
	return pt


def sphere_params_callback(params):
	#print(f'xc: {params.xc}, yc: {params.yc}, zc: {params.zc}, radius: {params.radius}')
	global estimated_x_pos
	global estimated_y_pos
	global estimated_z_pos
	
	estimated_x_pos = params.xc
	estimated_y_pos = params.yc
	estimated_z_pos = params.zc



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
		ball_in_camera_frame.point.x = estimated_x_pos
		ball_in_camera_frame.point.y = estimated_y_pos
		ball_in_camera_frame.point.z = estimated_z_pos

		
		# now convert the ball to the base frame:
		ball_in_base_frame = tfBuffer.transform(ball_in_camera_frame, 'base', rospy.Duration(1.0))
		
		if not plan_generated:
			
			# adjust starting pt to be a little closer to being above the ball, see if that helps...
			# TODO: add subscriber to ur5e/toolpose instead of manually setting the initial pt.
			#	do this outside of loop, only read once when you add the subscriber.
			initial_pt = create_new_pt(-0.014, -0.409, 0.274, 3.126, 0.0166, 1.5308)
			plan.points.append(initial_pt)
			
			above_ball_pt = create_new_pt(ball_in_base_frame.point.x, ball_in_base_frame.point.y, ball_in_base_frame.point.z + 0.1, 3.126, 0.0166, 1.5308)
			# TODO: add append step to create_new_pt fcn
			plan.points.append(above_ball_pt)
			
			# TODO: okay just add another intermediate pt here I guess.
			ball_center_pt = create_new_pt(ball_in_base_frame.point.x, ball_in_base_frame.point.y, ball_in_base_frame.point.z+0.02, 3.126, 0.0166, 1.5308)
			plan.points.append(ball_center_pt)
			
			# add back in other pts from simple planner once this is working.
			above_drop_pt = create_new_pt(ball_in_base_frame.point.x + 0.3, ball_in_base_frame.point.y + 0.1, ball_in_base_frame.point.z+0.2, 3.126, 0.0166, 1.5308)
			plan.points.append(above_drop_pt)
			
			drop_pt = create_new_pt(ball_in_base_frame.point.x + 0.3, ball_in_base_frame.point.y + 0.1, ball_in_base_frame.point.z+0.1, 3.126, 0.0166, 1.5308)
			
			plan.points.append(drop_pt)
			plan_generated = True
			
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
