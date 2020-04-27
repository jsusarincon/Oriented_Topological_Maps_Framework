//This node can activate the differente maneuvers of the robot using the keyboar:
// publishing on go_corridor 0 the robot stops, publishing 1 the robots follows the corridor slow, 2 fast
//publishing on go_cross_door 0 the robot stops, 1-4 cross door indicating if the door is E,N,W or S, from 5-8 is cross door fast.
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "testing_semantic_cmdp/logicalImage.h"
#include "testing_semantic_cmdp/collision.h"
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

using namespace std;

string modelName="";

void readObjectsDetected(const testing_semantic_cmdp::logicalImage::ConstPtr& msg){
  //cout <<"I heard: "<<msg->modelName<<endl;
  	
	
	modelName=msg->modelName; //model name 
	if (modelName.find("corridor") !=string::npos || modelName.find("se1") !=string::npos || modelName.find("intersection") !=string::npos){ //find the word corridor or se1 in the modelName and ignore the other objects
		modelName=msg->modelName; //model name plus compass reference
	}else{
		modelName="Other object";
	}
	//cout<<"modelName: "<<modelName<<endl;

}

void collisionDetected(const testing_semantic_cmdp::collision::ConstPtr& msg)
{
  cout <<"I heard: "<<msg->modelName<<endl;

  //sleep(20000); //Sleeps for 1000 ms or 1 sec
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "maneuvers_manually_node");
	ros::NodeHandle nh;
	ros::Publisher send = nh.advertise<std_msgs::Int16>("go_corridor", 1);
	ros::Publisher send2 = nh.advertise<std_msgs::Int16>("go_cross_door", 1);
	ros::Subscriber sub_objects = nh.subscribe("/objectsDetected", 1, readObjectsDetected);
	//ros::Subscriber sub_objects2 = nh.subscribe("collision", 1, collisionDetected);

	ros::Publisher send3 = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	std_msgs::Int16 msg1;
	msg1.data=0;
	int temp;
	char maneuver='n';
	time_t timer,timer2;
	double seconds=0.0;
	geometry_msgs::Twist msg;
 

	ros::Rate loop_rate(10);
	while (ros::ok)
	{

		//move until it doesnt see corridor or se1 to execute maneuvers
		while(modelName.find("corridor") !=string::npos || modelName.find("se1") !=string::npos){
			cout<<"out of the model: "<< modelName<<endl;
			msg.linear.x = 0.5;
			msg.angular.z = 0;
			send3.publish(msg);
			modelName="nothing";
			ros::spinOnce();
			loop_rate.sleep();
		}
		msg.linear.x = 0.0;
		msg.angular.z = 0;
		send3.publish(msg);

		//select which maneuver to use
		cout << "c: for corridor, d for door: ";
		while(maneuver=='n'){
			
			msg1.data=0;
			send.publish(msg1);
			send2.publish(msg1);
			cin >> maneuver;
			cout<<"maneuver "<<maneuver<<endl;
		}

		

		//execute maneuvers
		if (maneuver=='c'){
			cout << "1: for staying center slow, 2: for staying center fast: "<<endl;
			cin >> temp;
			if (temp==1 || temp==2){
				msg1.data=temp;
				cout<<"msg1.data "<<msg1.data<<endl;
				time(&timer);  /* get current time; same as: timer = time(NULL)  */
				while(modelName.find("corridor") ==string::npos ){
					cout<<"modelName 1: "<<modelName<<endl;
					send.publish(msg1);
					ros::spinOnce();
					loop_rate.sleep();
				}

				time(&timer2);  /* get current time; same as: timer = time(NULL)  */
				seconds+=difftime(timer2,timer);
				cout<<"time"<<seconds<<endl;

				//ros::Duration(1).sleep();
			}
		}else {
			if (maneuver=='d'){
				cout << "1-4 cross door indicating if the door is E,N,W or S SLOW"<<endl;
				cout << "5-8 cross door indicating if the door is E,N,W or S FAST"<<endl;
				cin >> temp;
				if (temp==1 || temp==2 || temp==3 || temp==4 || temp==5 || temp==6 || temp==7 || temp==8){
					msg1.data=temp;
					cout<<"msg1.data bbb "<<msg1.data<<endl;
					
					time(&timer);  /* get current time; same as: timer = time(NULL)  */
					while(modelName.find("corridor") ==string::npos && modelName.find("se1") ==string::npos){
						cout<<"modelName 2: "<<modelName<<endl;
						send2.publish(msg1);
						ros::spinOnce();
						loop_rate.sleep();
					}
					time(&timer2);  /* get current time; same as: timer = time(NULL)  */
					seconds+=difftime(timer2,timer);
				 	ros::spinOnce();
					loop_rate.sleep();
				}
			}
		}
		
		if (maneuver=='q'){
			return 0;
		}

		maneuver='n';
		
		cin.clear();
		ros::spinOnce();
		loop_rate.sleep();

	}
	 
  return 0;
}
