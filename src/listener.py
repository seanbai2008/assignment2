#!/usr/bin/python

# ME495 Embedded System for Robotics
# Assignment 2
# Name: Fan Bai
# Student ID: 2937200

# please check the launch file for the turtlesim, because I have worte my parameter and rosbag code in the launch file

import rospy
import roslib
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def listener():

    listener = tf.TransformListener()
    
    frequency = rospy.get_param("listener_frequency")
    
    pub = rospy.Publisher('visualization_marker',Marker,queue_size=100)   
    
    rate = rospy.Rate(frequency)
    
    rospy.sleep(0.5)
    
    marker = Marker()
    
    id = 0
    
    
    while not rospy.is_shutdown():
    

        (trans1,rot1) = listener.lookupTransform("base_link", "link3", rospy.Time(0))
        
        marker.header.frame_id = "base_link"
        
        marker.ns = "link3"
        
        marker.id += 1
        
        marker.header.stamp = rospy.Time()
             
        marker.type = Marker.SPHERE;
        
        marker.action = Marker.ADD
        
        marker.pose.position.x = trans1[0]
        
        marker.pose.position.y = trans1[1]
        
        marker.pose.position.z = trans1[2]
        
        marker.pose.orientation.x = rot1[0]
        
        marker.pose.orientation.y = rot1[1]
        
        marker.pose.orientation.z = rot1[2]
        
        marker.pose.orientation.w = rot1[3]
        
        marker.scale.x = 0.01
        
        marker.scale.y = 0.01
        
        marker.scale.z = 0.01
        
        marker.color.a = 1
        
        marker.color.r = 0.2
        
        marker.color.g = 0.3
        
        marker.color.b = 0.4
        
        marker.lifetime.secs = 2
               
        pub.publish(marker)
       
        rate.sleep()   
       

    
if __name__ == '__main__':
    try:
        rospy.init_node('Marker', anonymous=True)
        listener()
        rospy.spin()
    except rospy.ROSInterruptException: pass    
