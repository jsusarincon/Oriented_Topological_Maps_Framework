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
LINVEL=0.1	#how fast to advance
ANGVELFACTOR60=1.0	#how fast to rotate when obstacle is at 60 degrees
ANGVELFACTOR45=0.8 #how fast to rotate when obstacle is at 45 degrees
MAXANGVEL60=1.5
MAXANGVEL45=2
CONSTCLOSENESSALIGN=1.5 #the robot needs to be at least 1.5 meters away from the wall to cross the door correctly
ANGLEPERRAY=0.40625 #130/320 #130 degrees is 320 rays from laser

#Increasing this 2 values at the same time the robot goes faster but proportionally fast to avoid collision
#but when one is bigger than the other then the robot tends to crash
LINVELFACTOR=6
ANGVELFACTOR=6

alignDone=0
facingDoor=0	#robot facing the door
pointingDoor=0	#robot with angle to cross door
doorIndex=[0,0] #start and end of the door
goDoor=0 #when angle correct go forward to door
middleOfDoor=0
angleMiddleDoor=0.0
errorAngleDoor=0.3 #30cm to differentiate the hole of the door from the wall
errorAnglePointDoor=5 #5 degrees error when pointing to the door
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

	if alignDone==0 or alignDone==1:
		#doorPosition
		alignRobot(values)
	else:
		
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
				done =2
			#time.sleep(2)
			rospy.signal_shutdown("Robot is safe")


def alignRobot(values):
	global facingDoor,pointingDoor,doorIndex,goDoor,middleOfDoor,angleMiddleDoor,distanceToWall,alignDone

	#1) East goes from -10 to 10
	#2) North goes from 80 to 100
	#3) West goes from 170 to -170
	#4) South goes from -3/4*pi/4 to -pi/4
	if directionLastNode==1 and facingDoor==0: #facing south
	
		if directionRobotDeg >=-95 and directionRobotDeg<=-85:
			print "south", directionRobotDeg
			moveRobot(0.0,0.0) #CW
			facingDoor=1
		elif directionRobotDeg >-85 and directionRobotDeg<=0 or directionRobotDeg >0 and directionRobotDeg<=90:
			moveRobot(0.0,-0.2*ANGVELFACTOR) #CW
		elif directionRobotDeg <95 and directionRobotDeg>-180 or directionRobotDeg <=180 and directionRobotDeg>90:
			moveRobot(0.0,0.2*ANGVELFACTOR) #CW

	elif directionLastNode==4 and facingDoor==0: #facing south
	
		if directionRobotDeg >=-10 and directionRobotDeg<=10:
			print "east", directionRobotDeg
			facingDoor=1
		elif directionRobotDeg >10 and directionRobotDeg<=180:
			moveRobot(0.0,-0.2*ANGVELFACTOR) #CW
		elif directionRobotDeg >-180 and directionRobotDeg<-10:
			moveRobot(0.0,0.2*ANGVELFACTOR) #CCW

	if facingDoor==1 and pointingDoor==0:
		#measure where the door is
		doorIndex=[0,0]
		startDone=0
		pointingDoor=0
		for i in range(173,467): #from -60 to 60, all the front check where the door is
			#difference= abs(values[i]-values[i+1])
			#if math.isinf(difference) or math.isnan(difference):
			#	difference=errorAngleDoor*2
			if  math.isinf(values[i]) and math.isinf(values[i+1]):
				print "both INF"
			else:
				difference= abs(values[i]-values[i+1])
				print i, difference,startDone,values[i],values[i+1]
				if difference>errorAngleDoor and startDone==0: #if the difference bewteen 2 rays is more than 1m there is a hole
					doorIndex[0]=i+1 #start
					startDone=1;
					print "start", i+1
				elif difference>errorAngleDoor and i-1>doorIndex[0] and startDone==1:
					doorIndex[1]=i-1 #end
					#startDone=0;
					pointingDoor=1
					print "end",doorIndex
					middleOfDoor=int((doorIndex[1]+doorIndex[0])/2)
					angleMiddleDoor=abs(540-middleOfDoor)*ANGLEPERRAY
					print "point door a:", doorIndex, middleOfDoor,angleMiddleDoor,abs(directionRobotDeg),abs(angleMiddleDoor-abs(directionRobotDeg))
				else:
					distanceToWall=values[i]+errorAngleDoor #aprox dist to wall

	if pointingDoor==1 and goDoor==0:
		
		meanCenter=sum(values[300:340])/len(values[300:340])
		print "point door",doorIndex, middleOfDoor,angleMiddleDoor,abs(directionRobotDeg),abs(angleMiddleDoor-abs(directionRobotDeg)),meanCenter
		
		if middleOfDoor<320 and abs(angleMiddleDoor-abs(directionRobotDeg))>errorAnglePointDoor:# and meanCenter<distanceToWall:
			moveRobot(0.0,-0.2*ANGVELFACTOR) #CW
			print "cw"
		elif middleOfDoor>=320 and abs(angleMiddleDoor-abs(directionRobotDeg))>errorAnglePointDoor:# and meanCenter<distanceToWall:
			moveRobot(0.0,0.2*ANGVELFACTOR) #CCW
			print "ccw"
		elif abs(angleMiddleDoor-abs(directionRobotDeg))<errorAnglePointDoor or meanCenter>distanceToWall:
			moveRobot(0.0,0.0) 
			goDoor=1

	if goDoor==1:
		print "goDoor"
		mean60Left=sum(values[464:470])/len(values[464:470])
		mean60Right=sum(values[170:176])/len(values[170:176])
		print CONSTCLOSENESSALIGN,mean60Left,mean60Right,alignDone
		if mean60Left>CONSTCLOSENESSALIGN and mean60Right>CONSTCLOSENESSALIGN and alignDone==0:
			moveRobot(0.1*LINVELFACTOR,0.0) 
			print "1 go"
		elif mean60Left>CONSTCLOSENESSALIGN/2 and mean60Right>CONSTCLOSENESSALIGN/2 and alignDone==1:
			moveRobot(0.05*LINVELFACTOR,0.0) 
			print "2 go"
		else:
			if mean60Left<CONSTCLOSENESSALIGN/2 or mean60Right<CONSTCLOSENESSALIGN/2:
				alignDone+=1
			alignDone+=1
			facingDoor=0
			pointingDoor=0
			goDoor=0
			print "alignDoneeeeee:", alignDone 
			#moveRobot(0,0)



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

