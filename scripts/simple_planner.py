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
	loop_rate = rospy.Rate(10)

	# define a plan variable
	plan = Plan()
	
	# initial pts from his code:
	initial_pt = create_new_pt(-0.7, -0.23, 0.363, 1.57, 0.0, 0.0)
	plan.points.append(initial_pt)
	
	pt_2 = create_new_pt(-0.6, -0.23, 0.25, 1.57, 0.0, 0.0)
	plan.points.append(pt_2)
	
	

	while not rospy.is_shutdown():
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
