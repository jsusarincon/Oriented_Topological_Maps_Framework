#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <gazebo_msgs/SetModelState.h>
#include <cstdlib>
#include <time.h>
#include <fstream>
#include <random>
//#include "calculate_trans_prob/collision.h"
#include <tf/transform_datatypes.h>

using namespace std;

int NUMTEST=500;
bool collisionFlag=false;
int selectwWorld=1; //1=crossdoor, 2=corridor

void sleep(int mseconds)
{
    clock_t goal = mseconds + clock();
    while (goal > clock());
}
/*
void collisionDetected(const calculate_trans_prob::collision::ConstPtr& msg)
{
  cout <<"I heard: "<<msg->modelName<<endl;
  collisionFlag=true;
  sleep(20000); //Sleeps for 1000 ms or 1 sec
}
*/
int main(int argc, char **argv)
{
	

	ros::init(argc, argv, "calculate_trans_prob_node");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	//ros::Subscriber sub = n.subscribe("/collision", 1, collisionDetected);

	//saving the data to a file
	ofstream data;
	data.open("src/calculate_trans_prob/src/dataExperimentsTransProb"); 

	//Random Numbers Within a Specific Range
    //adding max 0.5 metters error around from starting point
	const float range_from  = -0.8;
    const float range_to    = 0.8;
    //adding max 45 degrees error around from starting point
    const float range_fromAng  = -3.1415/6;
    const float range_toAng    = 3.1415/6;

    std::random_device                  rand_dev;
    std::mt19937                        generator(rand_dev());
    std::uniform_real_distribution<float>  distr(range_from, range_to);
    std::uniform_real_distribution<float>  distrAng(range_fromAng, range_toAng);
    float errorAddedDist, errorAddedAng;
    
    //Running NUMTEST tests to calculate the failure probability
	for (int i = 0; i < NUMTEST; ++i)
	{
		cout << endl<<endl<<"***** NUMTEST *****: " << i<<endl<<endl<<endl; 
		//Random Numbers Within a Specific Range
    	//adding max 1 metter error around from starting point
    	//rand()%(max-min+1) + min;
    	errorAddedDist =  distr(generator);
    	errorAddedAng =  distrAng(generator);
    	tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0+errorAddedAng);  // Create this quaternion from roll/pitch/yaw (in radians)
   		q.normalize();

    	cout << errorAddedDist <<" "<< errorAddedAng <<endl;
		geometry_msgs::Pose start_pose;
		if (selectwWorld==1){
			start_pose.position.x = -2.5+errorAddedDist;
			start_pose.position.y = -0.7+errorAddedDist;
		}else{
			start_pose.position.x = 6;
			start_pose.position.y = errorAddedDist*0.8;
		}
		
		start_pose.position.z = 0.01;
		start_pose.orientation.x = q[0];
		start_pose.orientation.y = q[1];
		start_pose.orientation.z = q[2];
		start_pose.orientation.w = q[3];

		geometry_msgs::Twist start_twist;
		start_twist.linear.x = 0.0;
		start_twist.linear.y = 0.0;
		start_twist.linear.z = 0.0;
		start_twist.angular.x = 0.0;
		start_twist.angular.y = 0.0;
		start_twist.angular.z = 0.0;
		
		//---------------------------------------------------------------------
		gazebo_msgs::SetModelState setmodelstate;
		gazebo_msgs::ModelState modelstate;
		modelstate.model_name = "p3at"; 
		modelstate.reference_frame = "world"; //world
		modelstate.twist = start_twist;
		modelstate.pose = start_pose;
		setmodelstate.request.model_state = modelstate;
		if (client.call(setmodelstate))
		{
		  //ROS_INFO("BRILLIANT!!!");
		  ROS_INFO("robot position x=%f, y=%f",modelstate.pose.position.x,modelstate.pose.position.y);
		  //sleep(1000000); //Sleeps for 1000 ms or 1 sec
		if (selectwWorld==1){
		  system("rosrun cross_door cross_door_node.py ");
		}else{
			system("rosrun go_corridor go_corridor_node.py ");
		}
		  
		  //system("rosnode kill rosout crossing_door");              //stop
		}
		else
		{
		  ROS_ERROR("Failed to call service ");
		  return 1;
		}
	}
	data.close();
	//ros::spin();

  return 0;
}
