#!/usr/bin/env python3  
import rospy

import numpy
import math

import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():

    T1 = numpy.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.64, 0.64,0.0)), tf.transformations.translation_matrix((1.5,0.8,0.0)))
    
    
    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_frame"
    t1.child_frame_id = "object_frame"
    
    qr1 = tf.transformations.quaternion_from_matrix(T1)
    
    t1.transform.rotation.x = qr1[0]
    t1.transform.rotation.y = qr1[1]
    t1.transform.rotation.z = qr1[2]
    t1.transform.rotation.w = qr1[3]
    
    tr1 = tf.transformations.translation_from_matrix(T1)
    
    t1.transform.translation.x = tr1[0]
    t1.transform.translation.y = tr1[1]
    t1.transform.translation.z = tr1[2]
    
    br.sendTransform(t1)
    
    T2 = numpy.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(1.5, (0,1,0))), tf.transformations.translation_matrix((0.0,0.0,-2.0)))
   
    
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_frame"
    t2.child_frame_id = "robot_frame"
    
    qr2 = tf.transformations.quaternion_from_matrix(T2) 
    
    
    
    t2.transform.rotation.x = qr2[0]
    t2.transform.rotation.y = qr2[1]
    t2.transform.rotation.z = qr2[2]
    t2.transform.rotation.w = qr2[3]
   
    tr2 = tf.transformations.translation_from_matrix(T2)
       
    t2.transform.translation.x = tr2[0]
    t2.transform.translation.y = tr2[1]
    t2.transform.translation.z = tr2[2]
    
    br.sendTransform(t2)

    vec1 = tf.transformations.inverse_matrix(T2)
    vec2 = T1
    
    Op = numpy.array([[0],[0],[0],[1]])
    Rp = numpy.dot(numpy.dot(vec1,vec2),Op)
    
    vec3 = numpy.array([[1,0,0,0.3],[0,1,0,0],[0,0,1,0.3],[0,0,0,1]])
    vec3 = tf.transformations.inverse_matrix(vec3)
    
    Cp = numpy.dot(vec3,Rp)
    
    xaxis = [1,0,0]
    
    direction = [Cp[0][0], Cp[1][0], Cp[2][0]]
    


    a, b = (direction / numpy.linalg.norm(direction)), (xaxis/ numpy.linalg.norm(xaxis))
    axisrot = numpy.cross(a, b)
    c = numpy.dot(a, b)
    theta = -math.acos(c)
    
    
    
    
    
    
   
    qr3= tf.transformations.quaternion_about_axis(theta, axisrot)
    
   
    
    
    
    t3 = geometry_msgs.msg.TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "robot_frame"
    t3.child_frame_id = "camera_frame"
    t3.transform.translation.x = 0.3
    t3.transform.translation.y = 0.0
    t3.transform.translation.z = 0.3
    
    t3.transform.rotation.x = qr3[0]
    t3.transform.rotation.y = qr3[1]
    t3.transform.rotation.z = qr3[2]
    t3.transform.rotation.w = qr3[3]
    
 
    br.sendTransform(t3)

    

if __name__ == '__main__':
    rospy.init_node('PublishTransform')

    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publish_transforms()
        rate.sleep()
