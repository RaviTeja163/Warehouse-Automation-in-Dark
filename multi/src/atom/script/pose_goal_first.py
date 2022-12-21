#!/usr/bin/env python3
import rospy
import time
from tf.transformations import euler_from_quaternion
import math 
import geometry_msgs.msg
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist,Pose

class robot_goal:

    def __init__(self) -> None:
        self.pose_robot_x = 0.0
        self.pose_robot_y = 0.0
        self.robot_yaw = 0.0
        self.atom_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        self.lifter = rospy.Publisher('/amazon_robot/joint1_position_controller/command', Float64, queue_size=10)
        self.pose_robot_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.rate = rospy.Rate(10.0)

        self.turn_rad = 2.0
        self.step_size = 0.2

        self.vel_msg = Twist()
        self.lift_msg = Float64()

    def odom_callback(self,msg):
        self.pose_robot_x = msg.pose.pose.position.x
        self.pose_robot_y = msg.pose.pose.position.y
        self.pose_robot_rot = msg.pose.pose.orientation
        self.robot_yaw = euler_from_quaternion([self.pose_robot_rot.x,self.pose_robot_rot.y,self.pose_robot_rot.z,self.pose_robot_rot.w])[2]

    def wrapTo2PI(self,theta):
        '''Normalize an angle in radians to [0, 2*pi]
        '''
        return theta % (2.*math.pi)
    
    def distance(self,one,two) -> float:
        return math.dist(list(one),list(two))

    def steerangle(self,a_one,a_two) -> float:
        return math.atan2(a_two[1] - a_one[1] , a_two[0] - a_one[0])
    
    def lift(self, is_lift):
        time.sleep(5)
        self.lift_msg = Float64()
        if is_lift:
            self.lift_msg.data = 1.0
            self.lifter.publish(self.lift_msg)
        else:
            self.lift_msg.data = 0.0
            self.lifter.publish(self.lift_msg)
        self.rate.sleep()

    def move_to_goal(self, goal, is_reverse):
        time.sleep(4)
        if is_reverse:
            time.sleep(4)

        self.vel_msg = Twist()
        present_point = (self.pose_robot_x,self.pose_robot_y)
        goal_point = goal[len(goal)-1]
        loop_count = 0
        print('in', present_point, goal_point, math.dist(present_point, goal_point))
        
        while math.dist(present_point, goal_point) > 0.05:
            kp_l = 0.2
            kp_a = 1
            present_point = (self.pose_robot_x, self.pose_robot_y)
            next_point = goal[loop_count]

            present_angle = self.robot_yaw
            if is_reverse and loop_count ==0:
                present_angle += 3.14
            goal_angle = self.steerangle(present_point, next_point)
            angle_correction_wrap = self.wrapTo2PI(goal_angle) - self.wrapTo2PI(present_angle)
            angle_correction_no_wrap = goal_angle - present_angle

            if abs(angle_correction_wrap) < abs(angle_correction_no_wrap):
                angle_correction = angle_correction_wrap
            else:
                angle_correction = angle_correction_no_wrap

            distance_correction = math.dist(present_point, next_point)

            if distance_correction > 0.2 and abs(angle_correction) > 0.01:
                print('Correcting Angle: Goal- ', self.wrapTo2PI(round(goal_angle, 3)), ' Present- ', self.wrapTo2PI(round(present_angle, 3)), 'Correction- ', round(angle_correction, 3))
                print('No Wrap: Goal- ', round(goal_angle, 3), ' Present- ', round(present_angle, 3))
                self.vel_msg.linear.x = 0.02
                self.vel_msg.linear.y = 0
                self.vel_msg.linear.z = 0
                self.vel_msg.angular.x = 0
                self.vel_msg.angular.y = 0
                if abs(angle_correction) < 0.5:
                    kp_a = 0.5/abs(angle_correction)
                if is_reverse:
                    kp_a = 0.05/abs(angle_correction)
                    self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = kp_a * angle_correction

                self.atom_vel.publish(self.vel_msg)
            
            else:
                print('Angle Perfect: Goal- ', self.wrapTo2PI(round(goal_angle, 3)), ' Present- ', self.wrapTo2PI(round(present_angle, 3)))
                print('Correcting Distance: Goal- ', next_point, ' Present- ', present_point)

                # Linear velocity in the x-axis.
                if distance_correction < 0.4:
                    kp_l = 0.08/distance_correction
                if is_reverse:
                    kp_l /= 8
                if is_reverse and loop_count ==0:
                    kp_l *= -1
                self.vel_msg.linear.x = kp_l * self.distance(present_point, next_point)
                self.vel_msg.linear.y = 0
                self.vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
                self.vel_msg.angular.x = 0
                self.vel_msg.angular.y = 0
                self.vel_msg.angular.z = 0
                
                # Publishing our vel_msg
                self.atom_vel.publish(self.vel_msg)

            if loop_count < len(goal)-1 and math.dist(present_point,next_point) < 0.2:
                print('NEXT')
                loop_count += 1

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.atom_vel.publish(self.vel_msg)



if __name__ == "__main__":
    rospy.init_node('first_robot_goal_node')
    try:
        atom_robot = robot_goal()
        path = [(-2,0),(-2,0.9)]
        path1 = [(-2,-0.1),(1,0),(3,0),(5,0)]
        # atom_robot.move_to_goal([(4.5,3)], False)
        atom_robot.move_to_goal(path, False)
        atom_robot.lift(True)
        atom_robot.move_to_goal(path1, True)
        atom_robot.lift(False)
        atom_robot.move_to_goal([(4,0)], True)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('exception')