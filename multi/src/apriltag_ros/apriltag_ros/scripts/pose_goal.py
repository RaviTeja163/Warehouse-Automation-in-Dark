#!/usr/bin/env python3

# from turtle import distance, position
import rospy
import tf
from tf.transformations import euler_from_quaternion
import dubins
import math 
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist,Pose

class robot_goal:

    def __init__(self) -> None:
        # self.pose_robot = np.zeros(3)
        # self.pose_robot_rot = np.zeros(4)
        self.pose_robot_x = 0.0
        self.pose_robot_y = 0.0
        self.robot_yaw = 0.0
        self.atom_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        self.pose_robot_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.rate = rospy.Rate(10.0)

        self.turn_rad = 2.0
        self.step_size = 0.2

        self.vel_msg = Twist()

    def odom_callback(self,msg):

        self.pose_robot_x = msg.pose.pose.position.x
        self.pose_robot_y = msg.pose.pose.position.y
        # self.pose_robot_x = msg.pose.pose.position.x
        self.pose_robot_rot = msg.pose.pose.orientation
        self.robot_yaw = euler_from_quaternion([self.pose_robot_rot.x,self.pose_robot_rot.y,self.pose_robot_rot.z,self.pose_robot_rot.w])[2]
        # print(self.pose_robot,self.pose_robot_rot)
        # print(self.robot_yaw)
        # print(self.pose_robot_x)

    def dubins_path(self,start,goal):

        path = dubins.shortest_path(start, goal, self.turn_rad)

        return path

    def distance(self,one,two) -> float:
        return math.dist(list(one),list(two))

    def steerangle(self,a_one,a_two) -> float:
        return math.atan2(a_two[1] - a_one[1] , a_two[0] - a_one[0])
    
    def move_to_goal(self,goal):
        initial = (0,0,0)
        path = self.dubins_path(initial,goal)
        waypoints, _ = path.sample_many(self.step_size)
        # print(waypoints)
        self.vel_msg = Twist()
        present_point = (self.pose_robot_x,self.pose_robot_y)
        goal_point = goal[0],goal[1]
        loop_count = 0

        while math.dist(present_point,goal_point) > 0.1:
            
            kp_l = 0.1
            kp_a = 0.1
            # point = waypoints[0] 
            present_point = (self.pose_robot_x,self.pose_robot_y)
            next_point = (waypoints[loop_count][0],waypoints[loop_count][1])
            print('present',present_point)
            print('next_point',next_point)
            # Linear velocity in the x-axis.
            self.vel_msg.linear.x = kp_l * self.distance(present_point,next_point)
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            # print(type())
            self.vel_msg.angular.z = kp_a * (self.steerangle(present_point,next_point) - self.robot_yaw )
            print(self.steerangle(present_point,next_point), self.robot_yaw)
            # Publishing our vel_msg
            self.atom_vel.publish(self.vel_msg)

            if math.dist(present_point,next_point) < 0.4:
                print('incresed')
                loop_count = loop_count+1

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.atom_vel.publish(self.vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('robot_go_goal_node')
    try:
        atom_robot = robot_goal()
        atom_robot.move_to_goal((3,10,0))
        # rospy.spin()
    except rospy.ROSInterruptException:
        print('exception')