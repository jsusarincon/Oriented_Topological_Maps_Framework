#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <gazebo_msgs/SetModelState.h>
#include <cstdlib>
#include <time.h>
#include <fstream>
#include <random>
#include "testing_semantic_cmdp/collision.h"
#include <tf/transform_datatypes.h>
#include <pwd.h>

using namespace std;


void saveConfigurationFile(string goal1,string goal2,int timeToGoal, float failureProbability,int typeExplorationStrategy, string sharedFolder){
	
	 //get the home path if we want to save the files to the home folder
	const char *homedir;
	homedir=getenv("HOME");
	if ( homedir == NULL ) {
    	homedir = getpwuid(getuid())->pw_dir;
	}	
    string s1(homedir);
	string sharedFolderFull=s1+sharedFolder;
	//sharedFolderName="/media/sharedFolderLinuxSemantic/"; //you can define your own folder to share the files


	//saving the setupTest to a file
	fstream setupTest;
	
	string setupFileName=sharedFolderFull+"setupTest.txt";
	setupTest.open (setupFileName, std::fstream::out | std::fstream::trunc);
	setupTest << goal1<<endl;
	setupTest << goal2<<endl;
	setupTest << timeToGoal<<endl;
	setupTest << failureProbability<<endl;
	setupTest << typeExplorationStrategy<<endl;
	setupTest << sharedFolderFull<<endl;	
	setupTest.close();
}

int main(int argc, char **argv){

	

	//setup parameters
	string goal1=argv[1];
	cout <<"goal1 "<<goal1<<endl;
	string goal2=argv[2];
	cout <<"goal2 "<<goal2<<endl;
	float failureProbability=strtod(argv[3],NULL);
	cout <<"failureProbability "<<failureProbability<<endl;
	int typeExplorationStrategy=atof(argv[4]);
	cout <<"typeExplorationStrategy "<<typeExplorationStrategy<<endl;
	string sharedFolder=argv[5];
	cout <<"sharedFolder "<<sharedFolder<<endl;
	int NUMTEST=atof(argv[6]);
	cout <<"NUMTEST "<<NUMTEST<<endl;
	string startingPoint=argv[7];
	cout <<"startingPoint "<<startingPoint<<endl;
	int temporalDeadlines=atof(argv[8]);
	cout <<"temporalDeadlines "<<temporalDeadlines<<endl;

	ros::init(argc, argv, "testing_semantic_cmdp_node");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

	//CMDP variables
	vector<string> startingPositions;

	startingPositions.push_back("corridor_1a"); //corridor
	startingPositions.push_back("corridor_4h"); //corridor_20
	startingPositions.push_back("West_se104"); //carpet_4
	startingPositions.push_back("se115"); //carpet_13
	startingPositions.push_back("corridor_3a"); //corridor_4

	int startingSelectedAddition=0;
	if (startingPoint=="A")	
			startingSelectedAddition=0;
	else if (startingPoint=="B")
			startingSelectedAddition=2;
		else if (startingPoint=="C")
			startingSelectedAddition=4;
			else{
				cout<<"ERROR: Wrong value for the starting point, please verify your exploration launch file parameter <<startingPoint>>"<<endl;
				return -1;
			}


	int TDLSelectedAddition=0;
	switch (temporalDeadlines)	{
		case 680:
		case 2899:
		case 5628:
			TDLSelectedAddition=0;
			break;
		case 477:
		case 2030:
		case 3940:
			TDLSelectedAddition=3;
			break;
		case 272:
		case 1160:
		case 2251:
			TDLSelectedAddition=6;
			break;
		case 68:
		case 290:
		case 563:
			TDLSelectedAddition=9;
			break;
		default:
			cout<<"ERROR: Wrong value for the temporal deadline, please verify your exploration launch file parameter <<temporalDeadlines>>"<<endl;
			return -1;
	}
	
	cout<<"Selection Data test: "<<startingSelectedAddition<<" "<<TDLSelectedAddition<<endl;
    //Running NUMTEST tests to calculate the failure probability
	for (int i = startingPositions.size()-1-startingSelectedAddition; i >=0 ; i=i-2){
		for(int reduceTemporalDeadL=0+TDLSelectedAddition; reduceTemporalDeadL<=9;reduceTemporalDeadL=reduceTemporalDeadL+3){	
			for (int j=0; j<NUMTEST;j++){		
			
				cout<<endl<<endl<<" -------> NUMTEST :"<<j<<endl;

				tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);  // Create this quaternion from roll/pitch/yaw (in radians)
				q.normalize();


				geometry_msgs::Pose start_pose;
				
				switch ( i ){  
					case 0:  
						start_pose.position.x = -27.6;
						start_pose.position.y = 22.0; 
						saveConfigurationFile(goal1,goal2,(int)(5117-5117*reduceTemporalDeadL/10),failureProbability,typeExplorationStrategy,sharedFolder); 	//This value is the average of the time after the first round of experiments 
						break;  
					case 1:  
						q = tf::createQuaternionFromRPY(0, 0, 0);  // Create this quaternion from roll/pitch/yaw (in radians)
						q.normalize();
						start_pose.position.x = -19.6;
						start_pose.position.y = 5.8; 

						break;  
					case 2:  
						start_pose.position.x = -2.6;
						start_pose.position.y = 17.8; 
						saveConfigurationFile(goal1,goal2,(int)(2636-2636*reduceTemporalDeadL/10),failureProbability,typeExplorationStrategy,sharedFolder); 	//This value is the average of the time after the first round of experiments
						break;    
					case 3:  
						start_pose.position.x = 6.0;
						start_pose.position.y = 17.0; 
						break;  
					case 4:  
						start_pose.position.x = 18.5;
						start_pose.position.y = 20.00; 
						saveConfigurationFile(goal1,goal2,(int)(619-619*reduceTemporalDeadL/10),failureProbability,typeExplorationStrategy,sharedFolder); 	//This value is the average of the time after the first round of experiments
						break; 
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
				if (client.call(setmodelstate)){
					//cout<<"robot position x= "<<modelstate.pose.position.x<<" y= "<<modelstate.pose.position.y<<endl;
				
					cout<<"\n\n ***** RUNNING CREATE TOPOMAP***** "<<endl;
					system("rosrun create_topomap create_topomap_node ");
					
				}else{
					ROS_ERROR("Failed to call service ");
					return 1;
				}
			}
		}
	}


  return 0;
}
