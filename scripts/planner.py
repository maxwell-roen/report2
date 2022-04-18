#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf2_geometry_msgs

from tf.transformations import *
from geometry_msgs.msg import Quaternion
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
	#q_rot = Quaternion()	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# define a plan variable
	plan = Plan()
	
	# do everything in relation to the base frame?
	while not rospy.is_shutdown():
		# try to get updated transform between base and camera
		try:
			trans = tfBuffer.lookup_transform('base', 'camera_color_optical_frame', rospy.Time())
			
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('issue getting frames')
			loop_rate.sleep()
			continue
		
		# extract coords
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		
		# do we even need this? This never gets used anywhere?
		# extract quaternion, convert to RPY
		#q_rot = trans.transform.rotation
		# last comma causes argument unpacking? try with.
		#roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])
		
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
		
		# we now have everything we need to define the pts for the plan
		# try to get this working without worrying about angular coordinates atm.
		initial_pt = create_new_pt(-0.7, -0.23, 0.363, 1.57, 0.0, 0.0)
		plan.points.append(initial_pt)
		
		# go to ball center
		above_ball_pt = create_new_pt(ball_in_base_frame.point.x, ball_in_base_frame.point.y, ball_in_base_frame.point.z, 1.57, 0.0, 0.0)
		plan.points.append(above_ball_pt)
		
		# let's say the initial pt is above the drop pt, go back there
		plan.points.append(initial_pt)
	
		# then just go down to the drop pt
		drop_pt = create_new_pt(0.98, -0.54, 0.0, 1.57, 0.0, 0.0)
		plan.points.append(drop_pt)
		
		# then back to the initial pt
		plan.points.append(initial_pt)
			
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
