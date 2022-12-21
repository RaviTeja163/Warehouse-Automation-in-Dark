#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import euler_from_quaternion
import dubins
import math 
import geometry_msgs.msg
from nav_msgs.msg import Odometry
turn_rad = 2.0
step_size = 0.1

rospy.init_node('tf_node')

listener = tf.TransformListener()

listener.waitForTransform("/odom", "/robot_footprint", rospy.Time(), rospy.Duration(4.0))
atom_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
pose_robot = rospy.Subscriber('/odom',Odometry,odom_callback)
rate = rospy.Rate(10.0)

pose_robot = Odometry()

def odom_callback(msg):

    pose_robot = msg.pose.pose.position
    pose_robot_rot = msg.pose.pose.orientation
    
    print(pose_robot,pose_robot_rot)
while not rospy.is_shutdown():
    try:
        now = rospy.Time.now()
        listener.waitForTransform("/odom", "/robot_footprint", now, rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform("/odom", "/robot_footprint", now)
        # print(trans,rot)
        rot_euler = euler_from_quaternion(rot)
        # print(rot_euler[2])
        start_position = (trans[0],trans[1],rot_euler[2])
        end_position = (2,2,0.707)
        path = dubins.shortest_path(start_position, end_position, turn_rad)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    angular = 4 * math.atan2(trans[1], -trans[0])
    linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
    print(angular,linear)
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    atom_vel.publish(cmd)

    rate.sleep()