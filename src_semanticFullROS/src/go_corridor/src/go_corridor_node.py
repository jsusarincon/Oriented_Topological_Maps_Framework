#!/usr/bin/env python

import rospy 
import time
import math

import os

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from cross_door.msg import collision 


MINDISTANCESIDE=2
DISTEMERGENCYSTOP=0.5 # minimum distance before stoping
LINVEL=0.2	#how fast to advance
ANGVELFACTOR60=1.0	#how fast to rotate when obstacle is at 60 degrees
ANGVELFACTOR45=0.5 #how fast to rotate when obstacle is at 45 degrees
MAXANGVEL60=1.5
MAXANGVEL45=2
CONSTCLOSENESSALIGN=1.5 #the robot needs to be at least 1.5 meters away from the wall to cross the door correctly
ANGLEPERRAY=0.40625 #130/320 #130 degrees is 320 rays from laser

#Increasing this 2 values at the same time the robot goes faster but proportionally fast to avoid collision
#but when one is bigger than the other then the robot tends to crash
LINVELFACTOR=3
ANGVELFACTOR=3

alignDone=0
facingDoor=0	#robot facing the door
pointingDoor=0	#robot with angle to cross door
doorIndex=[0,0] #start and end of the door
goDoor=0 #when angle correct go forward to door
middleOfDoor=0
angleMiddleDoor=0.0
errorAngleDoor=0.3 #30cm
errorAnglePointDoor=10 #5 degrees error when pointing to the door
distanceToWall=0.0
done=0
#1) East goes from -pi/4 to pi/4
#2) North goes from pi/4 to pi*3/4
#3) West goes from pi*3/4 to -pi*3/4
#4) South goes from -3/4*pi/4 to -pi/4
directionRobotDeg=0.0

#Depending of the direction of the last node the robot should face the perpendicular 
#direction to find the door and cross it, for example, if the corridor goes from N-S then 
#if the room is on the left it means that to cross the door the robot needs to face east
directionLastNode=1 # west-east


def collisionDetected(msg):
	print "Collision!"
	print msg.modelName
	moveRobot(0,0)
	fileName = os.getenv("HOME")+'/dataExperimentsTransProb.txt'
	file = open(fileName,"a")   #appending mode, add new data, w option is for erasing file
	file.write("0\n") 
	file.close() 
	#time.sleep(2)
	rospy.signal_shutdown("Robot crashed")



def callback(header):
	global done
	#there are 640 rays,  320 is the center front ray
	values=header.ranges
	#print "Values:", header.ranges[320]
	#time.sleep(2)

	
	
	#flag to know if there is a clear path to go forward
	counter=0
	for i in range(130,513): #from -90 to 90, all the front has to be free
		if values[i]>5:
			counter+=1

	if counter<350:
		stayCenter(values)
		done=1
	#	print "centerrrrrrr ", sum(values[310:330])/len(values[310:330])
	else:
		moveRobot(0,0)
		print "Robot safe",done
		if done==1:
			fileName = os.getenv("HOME")+'/dataExperimentsTransProb.txt'
			file = open(fileName,"a")   #appending mode, add new data, w option is for erasing file
			file.write("1\n") 
			file.close()
			done =2;
		#time.sleep(2)
		rospy.signal_shutdown("Robot is safe")




def stayCenter(values):
	forward=LINVEL*LINVELFACTOR;
	centermean=sum(values[310:330])/len(values[310:330])
	
	if centermean<DISTEMERGENCYSTOP:
		#moveRobot(0, 0)
		print "emergency stop: ", centermean
		#time.sleep(2)
		#rospy.signal_shutdown("Robot crashed")
	else:
		#60 degrees checking, if distance is less than MINDISTANCESIDE
		mean60Left=sum(values[464:470])/len(values[464:470])
		mean60Right=sum(values[170:176])/len(values[170:176])
		mean45Left=sum(values[327:433])/len(values[327:433])
		mean45Right=sum(values[207:213])/len(values[207:213])
		print "means60: ",mean60Right, mean60Left
		print "means45: ",mean45Right, mean45Left
		rotation60a=0
		rotation60b=0
		rotationR45a=0
		rotationR45b=0

		#if math.isinf(mean60Left) or  math.isinf(mean60Right) or math.isinf(mean45Left) or math.isinf(mean45Right):

		#obstacle at 60 degrees
		if mean60Right<MINDISTANCESIDE:
			if 0.2/mean60Right>MAXANGVEL60:
				rotation60a=MAXANGVEL60
			else:
				rotation60a=mean60Right/10
			#moveRobot(forward,rotation)
			print "to the left 60: ",forward,rotation60a
		if mean60Left<MINDISTANCESIDE:
			if 0.2/mean60Left>MAXANGVEL60:
				rotation60b=-MAXANGVEL60
			else:
				rotation60b=-0.2/mean60Left
			#moveRobot(forward,rotation)
			print "to the right 60: ",forward,rotation60b
		
		#final rotation
		rotation=(rotation60a+rotation60b)
		
		# if rotation>0 and rotation<0.4:
		# 	rotation=0.2
		# if rotation<0 and rotation>-0.4:
		# 	rotation=-0.2
		print "final rotation60: ",  rotation

		#obstacle at 45 degrees
		if mean45Right<MINDISTANCESIDE:
			if 0.2/mean45Right>MAXANGVEL45:
				rotationR45a=MAXANGVEL45
			else:
				rotationR45a=0.2/mean45Right
			#moveRobot(forward,rotationR)
			print "to the left 45: ",forward,rotationR45a
		if mean45Left<MINDISTANCESIDE:
			if 0.2/mean45Left>MAXANGVEL45:
				rotationR45b=-MAXANGVEL45
			else:
				rotationR45b=-0.2/mean45Left
			#moveRobot(forward,rotationR)
			print "to the right 45: ",forward,rotationR45b

		rotationR=(rotationR45a+rotationR45b)

		# if rotationR>0 and rotationR<0.4:
		# 	rotationR=0.2
		# if rotationR<0 and rotationR>-0.4:
		# 	rotationR=-0.2
		print "final rotation45: ",  rotationR

		   #average of rotations
		if rotation!=0 and rotationR!=0:	
			moveRobot(forward,(rotation+rotationR))
			print "final : ",  (rotation+rotationR)
		elif rotation!=0 and rotationR==0:
			moveRobot(forward,rotation)
			print "1 f r : ",  forward,rotation

		elif rotation==0 and rotationR!=0:
			moveRobot(forward,rotationR)
			print "2 f r : ",  forward,rotationR

		if mean45Right>MINDISTANCESIDE and mean45Left>MINDISTANCESIDE and mean60Right>MINDISTANCESIDE and mean60Left>MINDISTANCESIDE:
			#forward=0.3
			rotation=0.0
			moveRobot(forward,rotation)
			print "forward",forward,rotation

		
		

def readDirection(directionRobotMsg):
	global directionRobotDeg
	directionRobotDeg=directionRobotMsg.data*180/math.pi
	#print directionRobotDeg

def moveRobot(linVel,rotVel):
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) #publisher velocity
	#rospy.loginfo(moveRobot)
	cmdMoveRobot=Twist()
	cmdMoveRobot.linear.x=linVel
	cmdMoveRobot.angular.z=rotVel
	pub.publish(cmdMoveRobot)


def listenerScan():
	rospy.init_node('crossing_door', anonymous=True)
	#time.sleep(2)
	rospy.Subscriber("/scan", LaserScan, callback)
	rospy.Subscriber("collision", collision, collisionDetected)
	rospy.Subscriber("direction_robot",Float64, readDirection)
	rospy.spin()

if __name__ == '__main__':
	listenerScan()

