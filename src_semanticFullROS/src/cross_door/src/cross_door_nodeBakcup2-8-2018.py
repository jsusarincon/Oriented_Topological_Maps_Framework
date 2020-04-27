#!/usr/bin/env python

import rospy 
import time
import math

import os

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from cross_door.msg import collision 


MINDISTANCESIDE=3
DISTEMERGENCYSTOP=0.5 # minimum distance before stoping
LINVEL=0.2	#how fast to advance
ANGVELFACTOR60=0.1	#how fast to rotate when obstacle is at 60 degrees
ANGVELFACTOR45=0.25 #how fast to rotate when obstacle is at 45 degrees
MAXANGVEL60=1
MAXANGVEL45=1.5

#Increasing this 2 values at the same time the robot goes faster but proportionally fast to avoid collision
#but when one is bigger than the other then the robot tends to crash
LINVELFACTOR=2
ANGVELFACTOR=2


values=[]
idx=0
cmdMoveRobot=Twist()
meanRightOld=0
meanRightDelta=-100
alignDone=0
measureDisWallFlag=1
closenessWall=0
parallelReady=0


def collisionDetected(msg):
	print "Collision!"
	print msg.modelName
	fileName = os.getenv("HOME")+'/dataExperimentsTransProb.txt'
	file = open(fileName,"a")   #appending mode, add new data, w option is for erasing file
	file.write("0\n") 
	file.close() 
	#time.sleep(2)
	rospy.signal_shutdown("Robot crashed")



def callback(header):
   
    global values
    global flag
    values=header.ranges
    #print "Values:", values[639]
    #time.sleep(2)
    #there are 640 rays,  320 is the center front ray
    
    #there are 2 possible positions for the door

	#1)						
	#-----------  ----		-----------------
	#			  robot 		robot
	#-----------------		----------  -----

	#2)						
	#-----------  ----		-----------------
	#	  robot 						 robot
	#-----------------		----------  -----

	#1) door on the right of the robot
	#the closeness to the wall depends of where the robot is, 
	#if it is close to the wall it should start rotating 
	#when less than 2 meters and if it is far then it 
	#rotates when more than 2 meters
    
    if alignDone==0:
    	#doorPosition
    	doorPosition=1
    	alignRobot(values,doorPosition)
    else:
		#flag to know if there is a clear path to go forward
	    counter=0
	    for i in range(130,513): #from -90 to 90, all the front has to be free
	    	if values[i]>5:
				counter+=1
		if counter<350:
			stayCenter(values)
			#print "centerrrrrrr"
		else:
			moveRobot(0,0)
			print "Robot safe"
			fileName = os.getenv("HOME")+'/dataExperimentsTransProb.txt'
			file = open(fileName,"a")   #appending mode, add new data, w option is for erasing file
			file.write("1\n") 
			file.close()
			#time.sleep(2)
			rospy.signal_shutdown("Robot is safe")


def alignRobot(values, doorPosition):
	global meanRightDelta,meanRightOld,alignDone,measureDisWallFlag,closenessWall,parallelReady

	#there are 2 possible positions for the door

	#1)						
	#-----------  ----		-----------------
	#			  robot 		robot
	#-----------------		----------  -----

	#2)						
	#-----------  ----		-----------------
	#	  robot 						 robot
	#-----------------		----------  -----

	#1) door on the right of the robot
	#the closeness to the wall depends of where the robot is, 
	#if it is close to the wall it should start rotating 
	#when less than 2 meters and if it is far then it 
	#rotates when more than 2 meters


	if doorPosition==1:
	
		
		checkParallelRobot=abs(values[70]-values[130])
		
		if checkParallelRobot>0.02 and parallelReady==0 and checkParallelRobot<20:
				if values[70]>values[130]:
					moveRobot(0,0.1)
				else:
					moveRobot(0,-0.1)
				print "parallel ",checkParallelRobot
		else:
			constCloseness=4
			if measureDisWallFlag==1:		
				#measure at 90 degrees the distance to the wall to know when to stop before start rotating
				measureDisWallFlag=0
				mean90Right=sum(values[97:103])/len(values[97:103])
				closenessWall=constCloseness+(mean90Right-0.5)
				if closenessWall>constCloseness or closenessWall<-constCloseness:
					closenessWall=constCloseness
			if closenessWall==constCloseness:
				mean13XRight=sum(values[20:26])/len(values[20:26])
				mean9XRight=sum(values[117:123])/len(values[117:123])
			else:
				mean13XRight=sum(values[0:6])/len(values[0:6])
				mean9XRight=sum(values[97:103])/len(values[97:103])
			
			meanCenter=sum(values[300:340])/len(values[300:340])
		
			#print closenessWall

			if meanRightDelta<closenessWall:
				if closenessWall==constCloseness:
					meanRightNew=sum(values[57:63])/len(values[57:63])
				else:
					meanRightNew=sum(values[20:26])/len(values[20:26])
				meanRightDelta=abs(meanRightNew-meanRightOld)
				meanRightOld=meanRightNew
				moveRobot(0.2*LINVELFACTOR,0)
				parallelReady=1 
			   	print "forward",meanRightDelta, meanRightOld, meanRightNew
			#when next to edge of the door rotate it to face the door 
			elif mean13XRight <closenessWall: 
				
				moveRobot(0.05*LINVELFACTOR,-0.3*ANGVELFACTOR)
			   	print "rotation11",mean13XRight
			#stop when the sensor at 90 degrees is bigger than 1
			elif mean9XRight<closenessWall: 
				moveRobot(0,-0.3*ANGVELFACTOR)
			   	print "rotation22",mean9XRight
			elif meanCenter>closenessWall and mean9XRight>closenessWall:
				rotation=0
				forward=0
				alignDone=1
				moveRobot(forward*LINVELFACTOR,rotation*ANGVELFACTOR)
				print "stop1"
	else:
		checkParallelRobot=abs(values[569]-values[509])
		print "parallel ",checkParallelRobot
		if checkParallelRobot>0.02 and parallelReady==0 and checkParallelRobot<20:
				if values[569]>values[509]:
					moveRobot(0,-0.1)
				else:
					moveRobot(0,0.1)
				print "parallel ",checkParallelRobot
		else:
			constCloseness=4
			if measureDisWallFlag==1:		
				#measure at 90 degrees the distance to the wall to know when to stop before start rotating
				measureDisWallFlag=0
				mean90Right=sum(values[536:542])/len(values[536:542])
				closenessWall=constCloseness+(mean90Right-0.5)
				if closenessWall>constCloseness or closenessWall<-constCloseness:
					closenessWall=constCloseness
			if closenessWall==constCloseness:
				mean13XRight=sum(values[613:619])/len(values[613:619])
				mean9XRight=sum(values[516:522])/len(values[516:522])
			else:
				mean13XRight=sum(values[633:639])/len(values[633:639])
				mean9XRight=sum(values[537:542])/len(values[537:542])
			
			meanCenter=sum(values[300:340])/len(values[300:340])
		
			#print closenessWall

			if meanRightDelta<closenessWall:
				if closenessWall==constCloseness:
					meanRightNew=sum(values[576:582])/len(values[576:582])
				else:
					meanRightNew=sum(values[613:619])/len(values[619:619])
				meanRightDelta=abs(meanRightNew-meanRightOld)
				meanRightOld=meanRightNew
				moveRobot(0.2*LINVELFACTOR,0)
				parallelReady=1 
			   	print "forward",meanRightDelta, meanRightOld, meanRightNew
			#when next to edge of the door rotate it to face the door 
			elif mean13XRight <closenessWall: 
				
				moveRobot(0.05*LINVELFACTOR,0.3*ANGVELFACTOR)
			   	print "rotation11",mean13XRight
			#stop when the sensor at 90 degrees is bigger than 1
			elif mean9XRight<closenessWall: 
				moveRobot(0,0.3*ANGVELFACTOR)
			   	print "rotation22",mean9XRight
			elif meanCenter>closenessWall and mean9XRight>closenessWall:
				rotation=0
				forward=0
				alignDone=1
				moveRobot(forward*LINVELFACTOR,rotation*ANGVELFACTOR)
				print "stop2"


def stayCenter(values):
	forward=LINVEL;
	centermean=sum(values[310:330])/len(values[310:330])
	
	if centermean<DISTEMERGENCYSTOP:
		#moveRobot(0, 0)
		print "emergency stop: ", centermean
		time.sleep(2)
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
	   

	   	#obstacle at 60 degrees
		if mean60Right<MINDISTANCESIDE:
			if math.pow(ANGVELFACTOR60,mean60Right)>MAXANGVEL60:
				rotation60a=MAXANGVEL60
			else:
				rotation60a=math.pow(ANGVELFACTOR60,mean60Right)
			#moveRobot(forward,rotation)
			print "to the left 60: ",forward,rotation60a
		if mean60Left<MINDISTANCESIDE:
			if math.pow(ANGVELFACTOR60,mean60Left)>MAXANGVEL60:
				rotation60b=-MAXANGVEL60
			else:
				rotation60b=-math.pow(ANGVELFACTOR60,mean60Left)
			#moveRobot(forward,rotation)
			print "to the right 60: ",forward,rotation60b
		
		#final rotation
		rotation=(rotation60a+rotation60b)/2
		print "final rotation60: ",  rotation

	   	#obstacle at 45 degrees
	   	if mean45Right<MINDISTANCESIDE:
			if math.pow(ANGVELFACTOR45,mean45Right)>MAXANGVEL45:
				rotationR45a=MAXANGVEL45
			else:
				rotationR45a=math.pow(ANGVELFACTOR45,mean45Right)
			#moveRobot(forward,rotationR)
			print "to the left 45: ",forward,rotationR45a
	   	if mean45Left<MINDISTANCESIDE:
			if math.pow(ANGVELFACTOR45,mean45Left)>MAXANGVEL45:
				rotationR45b=-MAXANGVEL45
			else:
				rotationR45b=-math.pow(ANGVELFACTOR45,mean45Left)
			#moveRobot(forward,rotationR)
			print "to the right 45: ",forward,rotationR45b
	    
		rotationR=(rotationR45a+rotationR45b)/2
		print "final rotation45: ",  rotationR

		   #average of rotations
		if rotation!=0 and rotationR!=0:	
		 	moveRobot(forward,(rotation+rotationR)/2)
		 	print "final : ",  (rotation+rotationR)/2
		elif rotation!=0 and rotationR==0:
			moveRobot(forward,rotation)
		elif rotation==0 and rotationR!=0:
			moveRobot(forward,rotationR)

		if mean45Right>MINDISTANCESIDE and mean45Left>MINDISTANCESIDE and mean60Right>MINDISTANCESIDE and mean60Left>MINDISTANCESIDE:
		   	#forward=0.3
		   	rotation=0.0
		   	moveRobot(forward,rotation)
		   	print "forward",forward,rotation
	

def moveRobot(linVel,rotVel):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) #publisher velocity
    #rospy.loginfo(moveRobot)
    cmdMoveRobot.linear.x=linVel
    cmdMoveRobot.angular.z=rotVel
    pub.publish(cmdMoveRobot)


def listenerScan():
    rospy.init_node('crossing_door', anonymous=True)
    #time.sleep(2)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.Subscriber("collision", collision, collisionDetected)
    rospy.spin()

if __name__ == '__main__':
    listenerScan()

