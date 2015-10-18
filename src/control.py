#!/usr/bin/python

# ME495 Embedded System for Robotics
# Assignment 2
# Name: Fan Bai
# Student ID: 2937200

# please check the launch file for the turtlesim, because I have worte my parameter and rosbag code in the launch file
 
import rospy
import roslib
roslib.load_manifest('assignment2')
import tf
from std_msgs.msg import String 
from geometry_msgs.msg import TransformStamped
import math 
import time
from sensor_msgs.msg import JointState 

def mover():

    l1 = 1
    
    l2 = 1
    
    T = rospy.get_param("T")
    
    w = 2*math.pi/T
     
    t0 = time.time()
    
    pub = rospy.Publisher('joint_states',JointState,queue_size=10)
    
    frequency = rospy.get_param("control_frequency")
   
    rate = rospy.Rate(frequency)
    
    joint_state = JointState()
    
    joint_state.name.append("joint1")
    
    joint_state.name.append("joint2")
    
    joint_state.name.append("joint3")

    joint_state.position.append(0) 
    
    joint_state.position.append(0)
    
    joint_state.position.append(0)
    
    listener = tf.TransformListener()
    
    br = tf.TransformBroadcaster()
    
    trans1 = (0,0,0)
    
    trans2 = (0,0,0)
    
    trans3 = (0,0,0)
    
    rot1 = (0,0,0)
    
    rot2 = (0,0,0)
    
    rot3 = (0,0,0)
    
    rospy.sleep(0.5)
    
    while not rospy.is_shutdown():
    
        t = time.time() - t0
        
        x = 0.5*math.cos(w*t)+1.25

        y = 0.5*math.sin(w*t)
        
        r = math.sqrt(x**2+y**2)
        
        a = (l1**2+l2**2-r**2)/(2*l1*l2)
        
        b = (r**2+l1**2-l2**2)/(2*l1*r)
        
        theta1 = math.acos(a)
        
        theta2 = math.acos(b)
        
        joint_state.position[0] = math.atan(y/x)-theta2
        joint_state.position[1] = math.pi - theta1
    
        stamp = rospy.get_rostime()            
           
        br.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0, 0, joint_state.position[0]),stamp, "link1","base_link")
        br.sendTransform((1,0,0),tf.transformations.quaternion_from_euler(0, 0, joint_state.position[1]),stamp, "link2","link1")
        br.sendTransform((1,0,0),tf.transformations.quaternion_from_euler(0, 0, 0),stamp, "link3","link2")
        
        pub.publish(joint_state)   

        rate.sleep()   

if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_pub', anonymous=True)

        mover()
        rospy.spin()
    except rospy.ROSInterruptException: pass
