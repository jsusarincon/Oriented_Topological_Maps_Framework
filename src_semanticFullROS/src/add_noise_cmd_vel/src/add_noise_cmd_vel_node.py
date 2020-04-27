#!/usr/bin/env python

import rospy 
import math
import numpy as np

from geometry_msgs.msg import *

import matplotlib.pyplot as plt


pub = rospy.Publisher('noisy_cmd_vel', Twist, queue_size=1)

def get_truncated_normal(mean=0, sd=1, low=0, upp=10):
    return truncnorm((low - mean) / sd, (upp - mean) / sd, loc=mean, scale=sd)


def addNoise(velocityMsg):

    noisy_cmd_vel=velocityMsg

    velLin = velocityMsg.linear
    velAng = velocityMsg.angular

    #Random Numbers Within a Specific Range
    #adding max the 4th part of the speed from -vel/4 to vel/4
    a = -velLin.x/4
    b = velLin.x/4
    linRand = (b-a)*np.random.uniform(-1,1,1) + a

    #adding max the 4th part of the speed from -ang/4 to ang/4
    a = -velAng.z/4
    b = velAng.z/4
    angRand = (b-a)*np.random.uniform(-1,1,1) + a
    #s = np.random.normal(mean, Standard deviation , size) 
    #randNum = np.random.normal(0, 1,1)      
    #randNum2 = np.random.normal(0, 1,1)
    #s = np.random.uniform(-1,1,1000)
    #count, bins, ignored = plt.hist(s, 15, normed=True)
    #plt.plot(bins, np.ones_like(bins), linewidth=2, color='r')
    #plt.show()
    #for x in range(0,1000):
    #print linRand,    angRand  

    noisy_cmd_vel.linear.x+=linRand
    #noisy_cmd_vel.linear.y*=randNum   we dont move on y and z axis
    #noisy_cmd_vel.linear.z*=randNum
    
    #noisy_cmd_vel.angular.x*=randNum2  we dont rotate on roll and pitch axis
    #noisy_cmd_vel.angular.y*=randNum2
    noisy_cmd_vel.angular.z+=angRand

    #print noisy_cmd_vel
    pub.publish(noisy_cmd_vel)

def begin():
    rospy.init_node('add_noise_cmd_vel', anonymous=True) #make node 
    rospy.Subscriber('cmd_vel',Twist,addNoise)
    


if __name__ == "__main__":
    begin()
    rospy.spin() # not really necessary because we have while not rospy.is_shutdown()

