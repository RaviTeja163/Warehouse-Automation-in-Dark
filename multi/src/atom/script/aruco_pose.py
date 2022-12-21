#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped
import cv2
import numpy as np
# import math
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_from_matrix 

f_x = 9.197818005410206297e+02 
f_y = 9.164191196330992852e+02

c_x = 6.412645238542337438e+02
c_y = 3.370903598908081449e+02

matrix_camera = np.array( [[f_x, 0, c_x],
                 [0, f_y, c_y],
                 [0, 0, 1]], dtype = "double"
                         )

dist = np.array([2.332378846782352799e-01,8.667190294058459976e-01,-7.800953606437266950e-03,5.792148809203224611e-04,-8.460537431637392691e-01])

vid = vid = cv2.VideoCapture(0)

#ros stuff
pose_pub = rospy.Publisher('aruco_pose', PoseStamped, queue_size=1)
rospy.init_node('aruco_node_raw', anonymous=False)
rate = rospy.Rate(10) # 10hz
aruco_rot = PoseStamped()

while(True):
    
    ret, image = vid.read()
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if ret == True:
        
        markerSize = 5
        totalMarkers = 250

        key = getattr(cv2.aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
        arucoDict = cv2.aruco.Dictionary_get(key)

        # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5x5_250)

        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners_aruco, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict,parameters=arucoParams)

        print('ids',ids)

        frame_markers = cv2.aruco.drawDetectedMarkers(image.copy(), corners_aruco, ids)

        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners_aruco, 3, matrix_camera,dist)
        
        if ids is None:
             ids = []

        if len(ids)>0:

            # print(np.array(rvec))
            # Store the rotation information
            rotation_matrix = np.eye(4)
            rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[0]))[0]
            r = R.from_matrix(rotation_matrix[0:3, 0:3])
            quat = r.as_quat() 
            quat =  quaternion_from_matrix(r)  
            print(quat)
            print(r)
            aruco_rot.header.stamp = rospy.Time.now()
            aruco_rot.header.frame_id = "odom"
            # Quaternion format     
            transform_rotation_x = quat[0] 
            transform_rotation_y = quat[1] 
            transform_rotation_z = quat[2] 
            transform_rotation_w = quat[3] 

            aruco_rot.pose.position.x = tvec[0][0][0]
            aruco_rot.pose.position.y = tvec[0][0][1]
            aruco_rot.pose.position.z = tvec[0][0][2]

            # transform_translation_x = tvecs[0][0][0]
            # transform_translation_y = tvecs[0][0][1]
            # transform_translation_z = tvecs[0][0][2]

            aruco_rot.pose.orientation.x = 0.0
            aruco_rot.pose.orientation.y = 0.0
            aruco_rot.pose.orientation.z = 0.0
            aruco_rot.pose.orientation.w = 0.0
            
            # Euler angle format in radians
            #roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x,transform_rotation_y,transform_rotation_z,transform_rotation_w)
                
            # roll_x = math.degrees(roll_x)
            # pitch_y = math.degrees(pitch_y)
            # yaw_z = math.degrees(yaw_z)

            pose_pub.publish(aruco_rot)
            rate.sleep()

            cv2.imshow('frame', frame_markers)
            # print(transform_rotation_x,transform_rotation_y,transform_rotation_z,transform_rotation_w)
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
