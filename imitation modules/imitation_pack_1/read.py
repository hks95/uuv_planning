#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf as tfp
import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
from nav_msgs.msg import Odometry

rospy.init_node('Yeeho_tf_listener')
euler_from_quaternion = tfp.transformations.euler_from_quaternion
listener = tfp.TransformListener()    
rate = rospy.Rate(10.0)

poseX = 0.0
poseY = 0.0
poseZ = 0.0
orientX = 0.0
orientY = 0.0
orientZ = 0.0

def odometryCb(msg):
    global poseX
    global poseY 
    global poseZ
    global orientX
    global orientY 
    global orientZ

    poseX = msg.pose.pose.position.x #Front is positive
    poseY = msg.pose.pose.position.y#Left is positive
    poseZ = msg.pose.pose.position.z#Left is positive

    rot = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    orientX = euler_from_quaternion(rot)[0]#in3.14, increase counterclockwise
    orientY = euler_from_quaternion(rot)[1]#in3.14, increase counterclockwise
    orientZ = euler_from_quaternion(rot)[2]#in3.14, increase counterclockwise 
    

print("MAIN CODE READY")
rospy.Subscriber('/uuv_imu_pose',Odometry,odometryCb)

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('rexrov/base_link', 'world', rospy.Time(0))
        # (trans2,rot2) = listener.lookupTransform('rexrov/base_link_imu', 'rexrov/base_link', rospy.Time(0))
            
    except (tfp.LookupException, tfp.ConnectivityException, tfp.ExtrapolationException):
        continue

    #Read Data
    # poseX = trans[0]#Front is positive
    # poseY = trans[1]#Left is positive
    # poseZ = trans[2]#Left is positive

    # orientX = euler_from_quaternion(rot)[0]#in3.14, increase counterclockwise
    # orientY = euler_from_quaternion(rot)[1]#in3.14, increase counterclockwise
    # orientZ = euler_from_quaternion(rot)[2]#in3.14, increase counterclockwise       
    #print("Currently at", format(poseX,'.3f'), format(poseY,'.3f'), format(poseZ,'.3f'),"heading", format(orientX,'.3f'), format(orientY,'.3f'), format(orientZ,'.3f'))			
	
    #Calculate
    input_vector = np.array([[poseX,poseY,poseZ,orientX,orientY,orientZ]])
    print('VAL : ', input_vector)

    rate.sleep()#-2.68,front bumper-2.331
        
            
        
        
#IF ROSPY IS SHUTDOWN
print("rospy shutdown")
