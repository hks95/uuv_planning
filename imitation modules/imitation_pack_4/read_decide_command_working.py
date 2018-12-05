#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf as tfp
import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
import tensorflow as tf
from nav_msgs.msg import Odometry
import pdb
learning_rate = 0#Just as dummy variable
#import turtlesim.srv

#Load Neural Network Here

weight_directory = "./dec3/"

tf.reset_default_graph()
states = tf.placeholder(tf.float32,[None,6])
actions = tf.placeholder(tf.float32,[None,3])

layer_0 = tf.layers.dense(inputs=states,units = 128,activation=tf.nn.sigmoid)
layer_1 = tf.layers.dense(inputs=states,units = 128,activation=tf.nn.sigmoid)
layer_2 = tf.layers.dense(inputs=states,units = 128,activation=tf.nn.sigmoid)
layer_3 = tf.layers.dense(inputs=states,units = 128,activation=tf.nn.sigmoid)
output = tf.layers.dense(inputs=layer_3,units = 3,activation=tf.nn.tanh)

loss = tf.losses.mean_squared_error(labels=actions,predictions=output)
optimizer = tf.train.AdamOptimizer(learning_rate = learning_rate).minimize(loss) 

#Initialize
config = tf.ConfigProto(allow_soft_placement = True)
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
sess.run(tf.global_variables_initializer())
print('network generation complete!!')

#Load Weights
saver = tf.train.Saver()
saver.restore(sess, tf.train.latest_checkpoint(weight_directory))
print("Weights Loaded")


rospy.init_node('Yeeho_tf_listener')
euler_from_quaternion = tfp.transformations.euler_from_quaternion
listener = tfp.TransformListener()    
rate = rospy.Rate(10.0)

# vel = rospy.Publisher('/rexrov/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
thrust_pub = rospy.Publisher( '/rexrov/thruster_manager/input_stamped', geometry_msgs.msg.WrenchStamped, queue_size=1)
force_msg = geometry_msgs.msg.WrenchStamped()

# cmd = geometry_msgs.msg.Twist()
# cmd.linear.x =0
# cmd.linear.y = 0
# cmd.linear.z = 0
# cmd.angular.x = 0
# cmd.angular.y = 0
# cmd.angular.z = 0

poseX = 0.0
poseY = 0.0
poseZ = 0.0
orientX = 0.0
orientY = 0.0
orientZ = 0.0

prev_vel_command = np.zeros((1,6))
wrench = np.zeros((6,1))
control_saturation = 5000 #from the main launch file

def publish_control_wrench(cur_vel_command):
    global prev_vel_command 
    # pdb.set_trace()
    cur_vel = np.zeros((1,6))
    cur_vel[0,0] = cur_vel_command[0,0] 
    cur_vel[0,1] = cur_vel_command[0,1] 
    cur_vel[0,2] = cur_vel_command[0,2]

    accel_command = (cur_vel - prev_vel_command)/0.1
    accel_command = np.transpose(accel_command)
    mass = 1862.87
    wrench[0:3] = np.multiply(mass,accel_command[0:3])
    inertia_mat = np.array(((525.39,1.44,33.41),(1.44,794.20,2.6),(33.41,2.6,691.23)))
    wrench[3:6] = np.matmul(inertia_mat,accel_command[3:6])

    ############### Apply saturation
    for i in range(6):
        if wrench[i] < -control_saturation:
            wrench[i] = -control_saturation
        elif wrench[i] > control_saturation:
            wrench[i] = control_saturation


    force_msg.header.stamp = rospy.Time.now()
    force_msg.header.frame_id = 'rexrov/base_link'
    force_msg.wrench.force.x = wrench[0]
    force_msg.wrench.force.y = wrench[1]
    force_msg.wrench.force.z = wrench[2]

    force_msg.wrench.torque.x = 0 # 10*wrench[3]
    force_msg.wrench.torque.y = 0 # 10*wrench[4]
    force_msg.wrench.torque.z = 0 # 10*wrench[5]

    prev_vel_command = cur_vel
    print("cmd_force : ", wrench)

    thrust_pub.publish(force_msg)

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

print("MAIN CODE READY")
while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('rexrov/base_link', 'world', rospy.Time(0))
            
    except (tfp.LookupException, tfp.ConnectivityException, tfp.ExtrapolationException):
        continue
        
    #Read Data
    input_vector = np.array([[poseX,poseY,poseZ,orientX,orientY,orientZ]])
    #print(input_vector, input_vector.shape)
    command = sess.run(output, feed_dict = {states : input_vector})
    # cmd.linear.x = command[0,0]
    # cmd.linear.y = command[0,1]
    # cmd.linear.z = command[0,2]
    # cmd.angular.x = command[0,3]
    # cmd.angular.y = command[0,4]
    # cmd.angular.z = command[0,5]
    print(command)
    # vel.publish(command)        
    publish_control_wrench(command)
    rate.sleep()#-2.68,front bumper-2.331
        
            
        
        
#IF ROSPY IS SHUTDOWN
print("rospy shutdown")
