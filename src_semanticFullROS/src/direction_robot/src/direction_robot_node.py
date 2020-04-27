#!/usr/bin/env python

import rospy 
import math
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pub = rospy.Publisher('direction_robot', Float64,queue_size=1)

def transformation(tf_data):
    global pub

    (roll, pitch, yaw) = euler_from_quaternion(tf_data)  #(roll, pitch, yaw)
    #print yaw #x is red and yaw =0, and it goes up to pi which is -x axis, then from -pi to 0 again CCW 
    pub.publish(yaw)

def position(odom_data):

    pose = odom_data.pose.pose #  the x,y,z pose and quaternion orientation

    #print pose, "\n"

    direction=odom_data.pose.pose.orientation

    transformation([direction.x, direction.y, direction.z, direction.w])

def begin():
    rospy.init_node('direction_robot', anonymous=True) #make node 
    rospy.Subscriber('odom',Odometry,position)
    


if __name__ == "__main__":
    begin()
    rospy.spin() # not really necessary because we have while not rospy.is_shutdown()

