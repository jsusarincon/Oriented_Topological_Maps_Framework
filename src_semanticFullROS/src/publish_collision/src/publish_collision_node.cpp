#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "ros/ros.h"
#include <gazebo_msgs/SetModelState.h>
#include <cstdlib>
#include <time.h>
#include <fstream>
#include <random>
#include "publish_collision/collision.h"

using namespace std;

ros::NodeHandle n_; 
ros::Publisher pub_collision = n_.advertise<std_msgs::Int32>("/collisionDetected", 1); 

void collisionDetected(const publish_collision::collision::ConstPtr& msg)
{

	cout <<"I heard: "<<msg->modelName<<endl;
 
}

int main(int argc, char **argv)
{
	

	ros::init(argc, argv, "testing_semantic_cmdp_node");
	ros::Subscriber sub_collision=n_.subscribe("/collision", 100, collisionDetected);
	
	
	ros::spin();

  return 0;
}
