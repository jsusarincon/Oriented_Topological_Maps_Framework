#!/usr/bin/env python

import rospy 
import math
import numpy as np

from std_msgs.msg import Float64

import matplotlib.pyplot as plt


pub = rospy.Publisher('noisy_compass', Float64, queue_size=1)

def get_truncated_normal(mean=0, sd=1, low=0, upp=10):
    return truncnorm((low - mean) / sd, (upp - mean) / sd, loc=mean, scale=sd)


def addNoise(compassMsg):

    noisy_compass=compassMsg

    compassVal = compassMsg.data
    

    #Random Numbers Within a Specific Range
    #adding max the 4th part of the speed from -vel/4 to vel/4
    a = -3.1415/36  #5 degrees of error for the compass
    b = 3.1415/36
    compRand = (b-a)*np.random.uniform(-1,1,1) + a

    
    noisy_compass.data+=compRand


    #print noisy_compass
    pub.publish(noisy_compass)

def begin():
    rospy.init_node('add_noise_compass', anonymous=True) #make node 
    rospy.Subscriber('direction_robot',Float64,addNoise)
    


if __name__ == "__main__":
    begin()
    rospy.spin() # not really necessary because we have while not rospy.is_shutdown()

