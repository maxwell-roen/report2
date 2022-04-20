#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf2_geometry_msgs

from tf.transformations import *
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams


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





if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(3)

	# define a plan variable
	plan = Plan()
	
	# obtained from rostopic echo /ur5e/toolpose
	plan.points.append(create_new_pt(0.03, 0.52, 0.44, 3.08, 0.06, 0.19))
	plan.points.append(create_new_pt(0.03, 0.52, 0.09, 3.08, 0.06, 0.19))
	plan.points.append(create_new_pt(0.18, 0.52, 0.2, 3.08, 0.06, 0.19))
	plan.points.append(create_new_pt(0.18, 0.52, 0.09, 3.08, 0.06, 0.19))
	

	while not rospy.is_shutdown():
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
