#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "create_topomap_node.hh"
#include<limits>
#include <math.h>
#include <unistd.h>
#include <fstream>
#include <random>
#include "geometry_msgs/Twist.h"


#include <pwd.h>

#define _USE_MATH_DEFINES

DirectionalGraph::DirectionalGraph(){

    LEFTNUMBER=1;
    RIGHTNUMBER=2;
    getDirectionManually=0; //0 if we want automatic direction obtained from odom, 1 for manual human direction
	lastDirectionTaken=0;

	fatherChild.push_back(" ");
	fatherChild.push_back(" ");
	updateFatherChild=true;
	model_pos_x=0.0;
	model_pos_y=0.0;


    sub_objects = n_.subscribe("/objectsDetected", 100, &DirectionalGraph::readObjectsDetected, this);
    sub_direction=n_.subscribe("/noisy_compass", 100, &DirectionalGraph::readDirection, this);
	goCorridor = n_.advertise<std_msgs::Int16>("go_corridor", 1);
	goDoor = n_.advertise<std_msgs::Int16>("go_cross_door", 1);

	//sub_collision=n_.subscribe("/collision", 100, &DirectionalGraph::collisionDetected, this);
    pub_cmd_vel = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 

}


void DirectionalGraph::collisionDetected(const create_topomap::collision::ConstPtr& msg)
{
  cout <<"COLLISION!!! I heard: "<<msg->modelName<<endl;
  //collisionFlag=true;
  //ros::shutdown();
}


void DirectionalGraph::printVisitedLocations(){
	vector<string>  myTempvector;
	string tempNode;
	map <string, int> :: iterator itr;

	cout<<"\n \n print  visitedLocations"<<endl;
	for (int i=0; i<visitedLocations.size();i++){
		cout<<"Locations: "<<visitedLocations[i].name<<endl;
		cout<<"Edges: "<<endl;
		for (itr = visitedLocations[findFatherIndex(visitedLocations[i].name)].edges.begin(); itr != visitedLocations[findFatherIndex(visitedLocations[i].name)].edges.end(); ++itr){
				cout<<""<< itr->first <<"-"<< itr->second<<endl;
		}
		cout<<"visitedEdges: "<<endl;
		for (itr = visitedLocations[findFatherIndex(visitedLocations[i].name)].visitedEdges.begin(); itr != visitedLocations[findFatherIndex(visitedLocations[i].name)].visitedEdges.end(); ++itr){
				cout<<""<< itr->first <<"-"<< itr->second<<endl;
		}
		cout<<"connectedNodes: "<<endl;
		for (itr = visitedLocations[findFatherIndex(visitedLocations[i].name)].connectedNodes.begin(); itr != visitedLocations[findFatherIndex(visitedLocations[i].name)].connectedNodes.end(); ++itr){
				cout<<""<< itr->first <<"-"<< itr->second<<endl;
		}
	}
}

void DirectionalGraph::printGraph(){
	vector<string>  myTempvector;
	string tempNode;
	map <string, int> :: iterator itr;

	cout<<"\n \n print  visitedLocations"<<endl;
	for (int i=0; i<visitedLocations.size();i++){
		cout<<"visitedLocations: "<<visitedLocations[i].name<<endl;
	}

	cout << "print left graph"<<endl;
	for (vector< vector<string> >::iterator it=adjacencyList2Left.begin(); it !=adjacencyList2Left.end(); ++it){
		myTempvector=*it;
		for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			tempNode=*it2;
			cout <<" "<< tempNode << " #edges: "<<visitedLocations[findFatherIndex(tempNode)].visitedEdges.size();
			/*for (itr = visitedLocations[findFatherIndex(tempNode)].visitedEdges.begin(); itr != visitedLocations[findFatherIndex(tempNode)].visitedEdges.end(); ++itr){
				cout<<""<< itr->first <<"-"<< itr->second<<endl;
			}*/
		}
		cout <<endl;
	}
	cout << "print right graph"<<endl;
	for (vector< vector<string> >::iterator it=adjacencyList2Right.begin(); it !=adjacencyList2Right.end(); ++it){
		myTempvector=*it;
		for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			tempNode=*it2;
			cout<<" " << tempNode << " #edges: "<<visitedLocations[findFatherIndex(tempNode)].visitedEdges.size();
			/*for (itr = visitedLocations[findFatherIndex(tempNode)].visitedEdges.begin(); itr != visitedLocations[findFatherIndex(tempNode)].visitedEdges.end(); ++itr){
				cout<<""<< itr->first <<"-"<< itr->second<<endl;
			}*/
		}
		cout <<endl;
	}
}
void DirectionalGraph::saveListsToFile(){
	ofstream outfile1,outfile2,outfileMatrixLeft,outfileMatrixRight, outfileListNodes,outfileListNodesType ;
	const char *homedir;
	vector<string>  myTempvector;

	int matrixLeft[visitedLocations.size()][visitedLocations.size()]={};
	int matrixRight[visitedLocations.size()][visitedLocations.size()]={};
	map<string,int> dictionaryNodes;

	string s1("/media/sf_sharedFolderLinuxSemantic/");	
	string leftFileNameFinal=s1+"leftList.txt";
	string rightFileNameFinal=s1+"rightList.txt";
	string matrixLeftFileNameFinal=s1+"adjMatrixLeft.txt";
	string matrixRightFileNameFinal=s1+"adjMatrixRight.txt";
	string listNodesFileNameFinal=s1+"listNodes.txt";
	string listNodesTypeFileNameFinal=s1+"listNodesType.txt";

	outfile1.open(leftFileNameFinal.c_str());
	outfile2.open(rightFileNameFinal.c_str());
	outfileMatrixLeft.open(matrixLeftFileNameFinal.c_str());
	outfileMatrixRight.open(matrixRightFileNameFinal.c_str());
	outfileListNodes.open(listNodesFileNameFinal.c_str());
	outfileListNodesType.open(listNodesTypeFileNameFinal.c_str());
		
	//creating dictionary (map) of nodes with indexes
	for(int i=0; i<visitedLocations.size(); i++){
		
		dictionaryNodes[visitedLocations[i].name]=i;
		outfileListNodes << i+1 <<endl;
		if (visitedLocations[i].name.find("corridor") !=string::npos){
			outfileListNodesType <<'c'<<endl;	//corridor
		}else{
			outfileListNodesType <<'r'<<endl;	//room
		}

		//cout<<"visitedLocations[i] "<<visitedLocations[i] <<endl;
	}

	cout << "saving left graph"<<endl;
	int nodeIndexTemp=-1;
	int nodeIndexTemp2=-1;
	for(int i=0; i<adjacencyList2Left.size(); i++){
		myTempvector=adjacencyList2Left[i];
		nodeIndexTemp=dictionaryNodes[myTempvector[0]];
		for(int j=1; j<myTempvector.size(); j++){  				//we analize the nodes after the first one j=1 and not 0 because of that
			outfile1 << myTempvector[j] << " ";					//adjacency list
			nodeIndexTemp2=dictionaryNodes[myTempvector[j]];	//adjancency matrix
			//cout<< "size: "<<visitedLocations.size()<<" matrix left dic "<<nodeIndexTemp<<" "<<nodeIndexTemp2<<endl;
			matrixLeft[nodeIndexTemp][nodeIndexTemp2]=1;		//adjancency matrix
		}
		outfile1 << endl;	//adjacency list
	}
	cout << "saving right graph"<<endl;
	for(int i=0; i<adjacencyList2Right.size(); i++){
		myTempvector=adjacencyList2Right[i];
		nodeIndexTemp=dictionaryNodes[myTempvector[0]];
		for(int j=1; j<myTempvector.size(); j++){			//we analize the nodes after the first one j=1 and not 0 because of that
			outfile1 << myTempvector[j] << " "; 			//adjacency list
			nodeIndexTemp2=dictionaryNodes[myTempvector[j]]; //adjancency matrix
			matrixRight[nodeIndexTemp][nodeIndexTemp2]=1;	//adjancency matrix
		}
		outfile1 << endl;	//adjacency list
	}

	//printing matrices
	cout << "saving adjancency matrices"<<endl;

	for (int i=0; i<visitedLocations.size();i++){
		for(int j=0; j<visitedLocations.size(); j++){
			//cout<< "matrix left "<<i<<" "<<j<<": "<<matrixLeft[i][j]<<endl;
			outfileMatrixLeft<<matrixLeft[i][j]<<' ';
			outfileMatrixRight<<matrixRight[i][j]<<' ';
		}
		outfileMatrixLeft << endl;
		outfileMatrixRight << endl;
	}

	outfile1.close();
	outfile2.close();

	outfileMatrixLeft.close();
	outfileMatrixRight.close();

	outfileListNodes.close();
	outfileListNodesType.close();

	cout <<"Files Printed"<<endl;
}

/*
bool DirectionalGraph::findEdge(string node1, string node2){
	//cout <<"finding edge"<<endl;
	vector<string>  myTempvector;
	
    		//cout << "searching right graph"<<endl;
	for (vector< vector<string> >::iterator it=adjacencyList2Right.begin(); it !=adjacencyList2Right.end(); ++it){
		myTempvector=*it;
		if (myTempvector[0]==node1){
			for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
				if (*it2==node2){
					//cout<<"found: "<< *it2 <<endl;
					return true;
				}
			}
		}
	}

			//cout << "searching left graph"<<endl;
	for (vector< vector<string> >::iterator it=adjacencyList2Left.begin(); it !=adjacencyList2Left.end(); ++it){
		myTempvector=*it;
		if (myTempvector[0]==node1){
			
			for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
				
				if (*it2==node2){
					//cout<<"found: "<< *it2 <<endl;
					return true;
				}
			}
		}
	}   	

    return false; //if the edge couldnt be found
}
*/

void  DirectionalGraph::addEdge(string currentModelName, int *direction, int lastEdgeAdded, int lastNodeAddedIndex){
	
	int lOrR;
	int tempDirection[2];
	int newNodeAddedIndex;
	int tempIndex;
	vector<string> tempVector1,tempVector2;
	string oldNodeName;

	tempDirection[0]=direction[0];
	tempDirection[1]=direction[1];
	lOrR=direction[0];

	updateFatherChild=false; //we deactivate the update of  fatherchild vector until we finish adding the edge

cout<<"lOrR: "<<lOrR<<endl;
	if (lOrR==RIGHTNUMBER){
		cout<<"right right"<<endl;
		tempDirection[0]=LEFTNUMBER;
		newNodeAddedIndex=addNode(currentModelName,tempDirection);
		//update visitedEdges
		printVisitedLocations();
		updateVisitedEdges(direction);
		cout<<"updateVisitedEdges done"<<endl;
		//Update Left
		cout<<"Update Left "<<endl;
		oldNodeName=fatherChild[0];

		adjacencyList2Left[newNodeAddedIndex].push_back(oldNodeName);
		cout<<"Update left done"<<endl;
		//Update Right
		//complete list to make sure all the node are connected
		cout<<"Update Right"<<endl;
		
		for (int j=1;j<visitedLocations.size();j++){
			tempIndex=-1; 
			for(int i=0;i<adjacencyList2Right.size();i++){
				if (adjacencyList2Right[i][0]==visitedLocations[j].name){
					tempIndex=i;
					break;
				}
			}
		}
		if (tempIndex==-1){		
			tempVector1.push_back(currentModelName);
			adjacencyList2Right.push_back(tempVector1);
		}

		tempIndex=-1;
		for(int i=0;i<adjacencyList2Right.size();i++){
			if (adjacencyList2Right[i][0]==oldNodeName){
				tempIndex=i;
				break;
			}
		}
		if (tempIndex==-1){	//The node doest exist yet so a new row has to be created
			tempVector2.push_back(currentModelName);
			adjacencyList2Right.push_back(tempVector2);
		}else{
			adjacencyList2Right[tempIndex].push_back(currentModelName);
		}
		printGraph();
		printVisitedLocations();
		cout<<"Update Right done"<<endl;
		
	}else{
		if (lOrR==LEFTNUMBER){
			cout<<"left left"<<endl;
			tempDirection[0]=RIGHTNUMBER;
			newNodeAddedIndex=addNode(currentModelName,tempDirection);
			//update visitedEdges
			updateVisitedEdges(direction);

			//Update right
			//oldNodeName=adjacencyList2Left[lastNodeAddedIndex][0];
			oldNodeName=fatherChild[0];
			adjacencyList2Right[newNodeAddedIndex].push_back(oldNodeName);

			//Update left
			
			for (int j=1;j<visitedLocations.size();j++){
				tempIndex=-1; 
				for(int i=0;i<adjacencyList2Left.size();i++){
					if (adjacencyList2Left[i][0]==visitedLocations[j].name){
						tempIndex=i;
						break;
					}
				}
			}
			if (tempIndex==-1){
			
				tempVector1.push_back(currentModelName);
				adjacencyList2Left.push_back(tempVector1);
			}

			tempIndex=-1;
			for(int i=0;i<adjacencyList2Left.size();i++){
				if (adjacencyList2Left[i][0]==oldNodeName){
					tempIndex=i;
					break;
				}
			}
			if (tempIndex==-1){	//The node doest exist yet so a new row has to be created
				tempVector2.push_back(currentModelName);
				adjacencyList2Left.push_back(tempVector2);
			}else{
				adjacencyList2Left[tempIndex].push_back(currentModelName);
			}
		}
	}
	
	updateFatherChild=true;
}


int DirectionalGraph::findFatherIndex(string currentModelName){
	NodeSemantic myNode;
		//cout << "searching right graph"<<endl;
		for (int i=0;i<visitedLocations.size();i++){
			myNode=visitedLocations[i];
			if (myNode.name==currentModelName){
				return i;
			}
		}
	cout<<currentModelName<<" not found in visitedLocations"<<endl;
	stopRobot();	//if failure then stop the robot
	return -1; //if index not found
}

void DirectionalGraph::getDirectionIntersections(int *direction){

		if (mydirection_robot >=-M_PI/4 && mydirection_robot<=M_PI/4){
			direction[0]=2; //indicates that the new place is on the right of the robot
			direction[1]=1; //indicates that the robot is facing east
			
		}else{
			if (mydirection_robot >M_PI/4 && mydirection_robot<=3*M_PI/4){
				direction[0]=2; //indicates that the new place is on the right of the robot
				direction[1]=2; //indicates that the robot is facing north
				
			}else{
				if (mydirection_robot >3*M_PI/6 && mydirection_robot <=M_PI || mydirection_robot<=-3*M_PI/6 && mydirection_robot>=-M_PI){
					direction[0]=1; //indicates that the new place is on the left of the robot
					direction[1]=3; //indicates that the robot is facing west
					
				}else{
					if (mydirection_robot >-3*M_PI/4 && mydirection_robot<-M_PI/4){
						direction[0]=1; //indicates that the new place is on the left of the robot
						direction[1]=4; //indicates that the robot is facing south
						
					}
				}
			}
		}

}

void DirectionalGraph::getDirection(int *direction){

	//int direction[2]={}; //indicates if the new place can be placed on the left (1) of the right (2)

	if (getDirectionManually==1) { //human indicates the direction

		while (direction[0]<LEFTNUMBER || direction[0]>RIGHTNUMBER){ //we have to select if the new place is to the right or the left of the previous place. 
			cout << "please indicate the direction of the new place: 1=left, 2=right and press ENTER" <<endl;
			//cin >> direction;
			while(!(cin >> direction[0])){ //the user writes a character not a number
		        cin.clear();
		        cin.ignore(numeric_limits<streamsize>::max(), '\n');
		        cout << "Invalid input, only numbers please.  Try again: ";
	    	}
	    	 
		}
		cout << "You entered: " << direction[0] << endl;

	}else{	//get the direction automatically from the topic direction_robot that comes from odom 
		/*
		if robot pointing east turn 90 CW for convention of direction 

		if robot pointing north turn 90  CW for convention of direction 

		if robot pointint west turn 90 CCW for convention of direction 

		if robot pointing south turn 90 CCW for convention of direction 

		*Each direction is composed by 60 degrees (pi/6), 
		 0 is horizonal x axis that going CCW goes up to pi, 
		 then from horizonal negative x -pi goes to zero again 
					
			1) East goes from -pi/4 to pi/4
			2) North goes from pi/4 to pi*3/4
			3) West goes from pi*3/4 to -pi*3/4
			4) South goes from -3/4*pi/4 to -pi/4
		*/

	//cout<<"mydirection_robot: "<<mydirection_robot*180/M_PI<<endl;

		if (lastDirectionTaken!=0){ //use the direction provided by last direction taken unless the tag has the direction included like the rooms
				direction[0]=getlOrR(lastDirectionTaken);
				if (direction[0]!=1 && direction[0]!=2)
					cout<<"errorr directions: "<<direction[0]<<endl;
				direction[1]=lastDirectionTaken;
		}else{
			if (mydirection_robot >=-M_PI/18 && mydirection_robot<=M_PI/18){
				direction[0]=2; //indicates that the new place is on the right of the robot
				direction[1]=1; //indicates that the robot is facing east
				
			}else{
				if (mydirection_robot >8*M_PI/18 && mydirection_robot<=10*M_PI/18){
					direction[0]=2; //indicates that the new place is on the right of the robot
					direction[1]=2; //indicates that the robot is facing north
					
				}else{
					if (mydirection_robot >17*M_PI/18 && mydirection_robot <=M_PI || mydirection_robot<=-17*M_PI/18 && mydirection_robot>=-M_PI){
						direction[0]=1; //indicates that the new place is on the left of the robot
						direction[1]=3; //indicates that the robot is facing west
						
					}else{
						if (mydirection_robot >-10*M_PI/18 && mydirection_robot<-8*M_PI/18){
							direction[0]=1; //indicates that the new place is on the left of the robot
							direction[1]=4; //indicates that the robot is facing south
							
						}
					}
				}
			}
		}
	}
	//cout<<"direction: "<<direction[0] << " compass: " <<direction[1]<<endl;
	
}

int DirectionalGraph::addNode(string currentModelName, int *direction){
	cout<<"Adding a node..."<<endl;
	int lastNodeAddedIndex=-1;
	int lOrR;
	vector<string> tempVector;
	NodeSemantic newNode;
	newNode.name=currentModelName;
	newNode.direction=direction[1];
	newNode.finishedScaningEdgesFlag=false;

	visitedLocations.push_back(newNode);
	scanForEdges(direction[1],true,direction[0], currentModelName,visitedLocations.size()-1);
	stopRobot();
	lastNodeAddedIndex=visitedLocations.size()-1;
	
	lOrR=direction[0];
	tempVector.push_back(currentModelName);
	if (lOrR==LEFTNUMBER){
		adjacencyList2Left.push_back(tempVector);
		lastNodeAddedIndex=adjacencyList2Left.size()-1;
	}else{
		adjacencyList2Right.push_back(tempVector);
		lastNodeAddedIndex=adjacencyList2Right.size()-1;
	}
	
	return lastNodeAddedIndex;
}

void DirectionalGraph::updateFather(string currentModelName, int * direction){
	int lOrR;
	if (currentModelName.find("corridor") !=string::npos || currentModelName.find("se1") !=string::npos){ //find the word corridor or se1 in the modelName and ignore the other objects
			//lastFatherIndex=findFatherIndex( currentModelName);
			if (fatherChild[1]!=currentModelName && updateFatherChild==true){ //if current node (child) is already saved dont do anything	
				fatherChild[0]=fatherChild[1];	//father is the previous child
				fatherChild[1]=currentModelName; //child is the new node
			}
			//return currentModelName;
	}
}


void DirectionalGraph::scanForEdges(int initialOri,bool equalDirec,int lOrR,string myfirstModelName,int lastNodeAddedIndex){
	cout<<"Scaning edges..."<<endl;
	float currentOri;
	int direction[2]={};
	int finalDirection;
	int countDirections;
	//int directionPrecise[2]={};
	ros::Rate loop_rate(10);
	
	//move until it doesnt see corridor or se1 to execute maneuvers
	string outNode=myfirstModelName;

	moveMiddleTag();
	cout<<"Scaning edges middle done"<<endl;

	//rotate until the first orientation changes but still check if there is intersections
	string lastLettersFather,tempFather,lastLettersModelName, currentModelName;
	tempFather=fatherChild[1];
	//lastLettersFather=tempFather[tempFather.size()-3]+tempFather[tempFather.size()-2]+tempFather[tempFather.size()-1]+tempFather[tempFather.size()];
	for (auto it=tempFather.cend()-3; it!=tempFather.cend(); ++it)
		lastLettersFather+= *it;

	countDirections=0;
	while(equalDirec==true){
		getDirectionIntersections(direction);
		currentOri=direction[1];
		moveRobot(0,-0.4);
		
		if (currentOri!=0 ){
			
			if (intersectionsName.find("intersection") !=string::npos){
				
				currentModelName=intersectionsName;
				for (auto it=currentModelName.cend()-3; it!=currentModelName.cend(); ++it)
					lastLettersModelName+= *it;

				finalDirection=-1;
				if ( intersectionsName.find("East") !=string::npos &&  lastLettersModelName==lastLettersFather)
					finalDirection=1;
				if ( intersectionsName.find("South") !=string::npos &&  lastLettersModelName==lastLettersFather)
					finalDirection=4;
				if ( intersectionsName.find("West") !=string::npos &&  lastLettersModelName==lastLettersFather)
					finalDirection=3;
				if ( intersectionsName.find("North") !=string::npos &&  lastLettersModelName==lastLettersFather)
					finalDirection=2;
			
				lastLettersModelName="";

				if (finalDirection!=-1){
						visitedLocations[lastNodeAddedIndex].edges[intersectionsName]=finalDirection;
						visitedLocations[lastNodeAddedIndex].visitedEdges[intersectionsName]=0;

					}			
				// cout<<"Found a: "<<intersectionsName<<" direction: "<<finalDirection<<" father: "<<tempFather<<endl;				
			}
			if (initialOri-currentOri!=0){
				countDirections+=1;
			}else
				countDirections=0;

			cout<<"countDirectionsA: "<<countDirections<<endl;
			if (countDirections>20)
				break;
		}
		intersectionsName=" ";
		ros::spinOnce();
		loop_rate.sleep();
	}
	stopRobot();
	//rotate the rest of the 360 degrees to cover the whole area for possible intersections
	equalDirec=false;
	countDirections=0;
	while(equalDirec==false){
		getDirectionIntersections(direction);
		currentOri=direction[1];
		moveRobot(0,-0.4);
		//cout<<"scaning for edges 2: "<<intersectionsName<<endl;
		if (currentOri!=0 ){
			if (intersectionsName.find("intersection") !=string::npos){
					currentModelName=intersectionsName;
					for (auto it=currentModelName.cend()-3; it!=currentModelName.cend(); ++it)
						lastLettersModelName+= *it;

					finalDirection=-1;
					if ( intersectionsName.find("East") !=string::npos &&  lastLettersModelName==lastLettersFather)
						finalDirection=1;
					if ( intersectionsName.find("South") !=string::npos &&  lastLettersModelName==lastLettersFather)
						finalDirection=4;
					if ( intersectionsName.find("West") !=string::npos &&  lastLettersModelName==lastLettersFather)
						finalDirection=3;
					if ( intersectionsName.find("North") !=string::npos &&  lastLettersModelName==lastLettersFather)
						finalDirection=2;
				
					lastLettersModelName="";
					if (finalDirection!=-1){
						visitedLocations[lastNodeAddedIndex].edges[intersectionsName]=finalDirection;
						visitedLocations[lastNodeAddedIndex].visitedEdges[intersectionsName]=0;
						visitedLocations[lastNodeAddedIndex].finishedScaningEdgesFlag=true;
					}
					// cout<<"Found b: "<<intersectionsName<<" direction: "<<finalDirection<<" father: "<<tempFather<<endl;				
			}

			if (initialOri-currentOri==0){
				countDirections+=1;
			}else
				countDirections=0;
				
			cout<<"countDirectionsB: "<<countDirections<<endl;
			if (countDirections>20)
				break;
			
			ros::spinOnce();
			loop_rate.sleep();
		}
		intersectionsName=" ";
		ros::spinOnce();
		loop_rate.sleep();

	}
	faceNewDirection(initialOri); //locate the robot on the right orientation 
	stopRobot();
	ros::spinOnce();
	loop_rate.sleep();
}

void DirectionalGraph::readObjectsDetected(const create_topomap::logicalImage::ConstPtr& msg){
	int compass;
	int direction[2]={};

	if ((msg->modelName.find("corridor") !=string::npos || msg->modelName.find("se1") !=string::npos || msg->modelName.find("intersection") !=string::npos) ){ //find the word corridor or se1 in the modelName and ignore the other objects
		//modelName=msg->modelName+ss.str(); //model name plus compass reference
		if (msg->modelName.find("corridor") !=string::npos || msg->modelName.find("se1") !=string::npos){		
			modelName=msg->modelName;
			model_pos_x=msg->pose_pos_x;
			model_pos_y=msg->pose_pos_y;
			getDirection(direction);	
			updateFather(modelName,direction);
		}else{
			intersectionsName=msg->modelName;
		}
	}else{
		modelName="Other object";
		intersectionsName="Other object";
	}
	//cout<<"modelName: "<<modelName<<endl;
	
}

void DirectionalGraph::readDirection(const std_msgs::Float64::ConstPtr& msg){
  //cout <<"I heard: "<<msg->data<<endl;
  mydirection_robot=msg->data;
  
}

void DirectionalGraph::moveRobot(float lin, float ang){
	geometry_msgs::Twist msg;
	msg.linear.x = lin;
	msg.angular.z = ang;    
	pub_cmd_vel.publish(msg);
}

void DirectionalGraph::stopRobot(){
	ros::Rate loop_rate(10);
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.angular.z = 0;    
	pub_cmd_vel.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
}

void DirectionalGraph::faceNewDirection(int newDirection){

	cout<<"faceFront: "<<newDirection<<endl;
	ros::Rate loop_rate(10);
	//moveMiddleTag();

	if(newDirection==1){ 
		if (mydirection_robot>=0 && mydirection_robot<=M_PI)
			moveRobot(0,-0.4); //CW
		else
			moveRobot(0,0.4);	//CCW
	}else{
		if(newDirection==2){ 
			if (mydirection_robot>=-M_PI/2 && mydirection_robot<=M_PI/2)
				moveRobot(0,0.4);
			else
				moveRobot(0,-0.4);
		}else{
			if(newDirection==3){ 
				if (mydirection_robot>=0 && mydirection_robot<M_PI)
					moveRobot(0,0.4);
				else
					moveRobot(0,-0.4);
			}else{
				if (mydirection_robot<=M_PI/2 && mydirection_robot>-M_PI/2)
					moveRobot(0,-0.4);
				else
					moveRobot(0,0.4);
			}
		}
	}
	while (1){

		if (newDirection==1 && (mydirection_robot >=-M_PI/18 && mydirection_robot<=M_PI/18)){ //east
			stopRobot();
			break;
		}else{
			if (newDirection==2 && (mydirection_robot >8*M_PI/18 && mydirection_robot<=10*M_PI/18)){ //north
				stopRobot();
				break;
				
			}else{
				//west
				if (newDirection==3 && (mydirection_robot >17*M_PI/18 && mydirection_robot <=M_PI || mydirection_robot<=-17*M_PI/18 && mydirection_robot>=-M_PI)){
					stopRobot();
					break;
					
				}else{
					if (newDirection==4 && (mydirection_robot >-10*M_PI/18 && mydirection_robot<-8*M_PI/18)){ //south
						stopRobot();
						break;
					}
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	stopRobot();
	cout<<"faceFront done"<<endl;
}

int DirectionalGraph::opositeDirection(int currentDirection){
		cout<<"faceOff"<<endl;
		switch (currentDirection){
			case 1: 
				return 3;
			case 2: 
				return 4;
			case 3: 
				return 1;
			case 4: 
				return 2;
		}		
}

int DirectionalGraph::updateVisitedEdges(int *direction){
	map<string, int>::iterator it;
	map <string, int> :: iterator itr;
	int myOpositeDirection;
	int lOrR=direction[0];
	int numNodes=visitedLocations.size()-1;
	NodeSemantic oldNode,newNode;
	int oldNodeIndex,newNodeIndex;

	//update visitedEdges
	cout<<"update visitedEdges ttt"<<endl;
	oldNodeIndex=findFatherIndex( fatherChild[0]);
	newNodeIndex=findFatherIndex( fatherChild[1]);
	cout<<"update visitedEdges here: "<<oldNodeIndex<<" "<<newNodeIndex<<endl;
	if (oldNodeIndex==-1 || newNodeIndex==-1){
		cout<<"Error: finding indexes for father and child"<<endl;
		return -1;
	}
	oldNode=visitedLocations[oldNodeIndex];
	newNode=visitedLocations[newNodeIndex];

	cout<<endl<<"UPDATING VISITED"<<endl<<endl;

	//updating previous node
	cout<<"visitedLocations "<<numNodes<<endl;
	
	for (itr = oldNode.edges.begin(); itr != oldNode.edges.end(); ++itr){
		if (itr->second==newNode.direction){
			it = visitedLocations[oldNodeIndex].visitedEdges.find(itr->first); 
			//if (it != oldNode.edges.end())
			it->second = 1;
			visitedLocations[oldNodeIndex].connectedNodes[newNode.name]=itr->second;			
		}
	}
	
	//updating new node

	//if the robot is facing east then it means the edge west is visited because it is comming from the left
	myOpositeDirection=opositeDirection(newNode.direction);
	int counterEdges=0;
	for (itr = newNode.edges.begin(); itr != newNode.edges.end(); ++itr){
		if (itr->second==myOpositeDirection){
			counterEdges++;
			it = visitedLocations[newNodeIndex].visitedEdges.find(itr->first); 
			//if (it != newNode.edges.end())
			it->second = 1;
			visitedLocations[newNodeIndex].connectedNodes[oldNode.name]=itr->second;
			cout<<"first: "<<itr->first<<" second: "<<it->second<<endl;
		}
	}
	if (counterEdges==0){
		cout<<"error: no edges"<<endl;
		return -1;
	}
	
return 0; //no problems
}

//IT chooses for a edge the next direction to go
void DirectionalGraph::goNextEdge(NodeSemantic tempNode, int directionRobot){
	cout<<"Going random Edge"<<endl;
	
	ros::Rate loop_rate(10);
	updateFatherChild=false;

	int numEdges=tempNode.edges.size();
	stopRobot();
	if (numEdges!=0){
		map <string, int> :: iterator itr;
		vector<string>  namesEdges;
		vector<int>  directionEdges;
		for (itr = tempNode.edges.begin(); itr != tempNode.edges.end(); ++itr){
			namesEdges.push_back( itr->first);
			directionEdges.push_back(itr->second);
					//cout<< itr->first <<  '\t' << itr->second<<endl;
		}
		int pickedEdge;
		if (numEdges>1){
			while(1){
				random_device rd; // obtain a random number from hardware
				mt19937 eng(rd()); // seed the generator
				uniform_int_distribution<> distr(0, numEdges-1); // define the range

				pickedEdge=distr(eng);
				if (tempNode.visitedEdges[namesEdges[pickedEdge]]==0){ //if edge is not visited then use that edge, otherwise select another edge
					break;
					}
			}
		}else{
			pickedEdge=numEdges-1;
		}

		int newDirection = directionEdges[pickedEdge];


		if (newDirection!=tempNode.direction){ //if direction of the picked now is different from the current direction the robot needs to rotate
			faceNewDirection(newDirection);
		}
		std_msgs::Int16 goMsg;
		goMsg.data=1;
		string lastNodeSeen=fatherChild[1];
		cout<<"lastNodeSeen: "<<lastNodeSeen<<" newDirection: "<<newDirection<< " new node to go: " << namesEdges[pickedEdge]<<endl;
		//move to middle of tag
		moveMiddleTag();

		lastDirectionTaken=newDirection; //we save the direction we took to define the direction of new node
		updateFatherChild=true; //search for new fathers after being positioned located.
		
 		if (lastNodeSeen.find("corridor") !=string::npos && newDirection==directionRobot ) { //if robot is pointing
			cout<<"go corridor"<<endl;
			while(modelName.find("corridor") ==string::npos){
				//cout<<"modelName 1: "<<modelName<<endl;
				
				modelName=" ";
				goCorridor.publish(goMsg);
				
				ros::spinOnce();
				loop_rate.sleep();
				
			}
		}else{
			goMsg.data=newDirection;
			if (lastNodeSeen.find("se1") !=string::npos){
				cout<<"go door a"<<endl;
				
				while(modelName.find("corridor") ==string::npos ){
					//cout<<"modelName 2: "<<modelName<<endl;
				
					modelName=" ";
					goDoor.publish(goMsg);
					ros::spinOnce();
					loop_rate.sleep();
					
				}
			}
			
			if ( lastNodeSeen.find("corridor") !=string::npos && newDirection!=directionRobot){
				cout<<"go door b"<<endl;
				
				while((modelName.find("corridor") ==string::npos && modelName.find("se1") ==string::npos) ){
					//cout<<"modelName 3: "<<modelName<<endl;
					
					modelName=" ";
					goDoor.publish(goMsg);
					ros::spinOnce();
					loop_rate.sleep();
					
				}
			}
		}
		goMsg.data=0;
		goCorridor.publish(goMsg);
		goDoor.publish(goMsg);
	}else{
		cout<<"Error Node Without edges"<<endl;
	}

	updateFatherChild=false;
	//move to middle of tag
	moveMiddleTag();

	cout<<"Node reached and lastDirectionTaken: "<<lastDirectionTaken<<endl;
}


bool isFileExist(const char *fileName){
	ifstream file (fileName);
			
	if (file.is_open()){
		return true;
	}else{
		cout << "Unable to open file endWriting";
		return false; 
	}
}

int DirectionalGraph::getlOrR(int ENWS){
	if (ENWS==1)
		return 2;
	if (ENWS==2)
		return 2;
	if (ENWS==3)
		return 1;
	if (ENWS==4)
		return 1;
}

void DirectionalGraph::moveMiddleTag(){
	cout<<"Moving to the middle..."<<endl;
	ros::Rate loop_rate(10);
	float distanceModel=10.0;
	while((modelName.find("corridor") !=string::npos || modelName.find("se1") !=string::npos) && modelName==fatherChild[1]  && distanceModel>0.2){
		//cout<<"out of the model: "<< modelName<<endl;
		distanceModel=sqrt(pow(model_pos_x,2)+pow(model_pos_y,2));	
		cout<<"distanceModel: "<< distanceModel<<" "<<modelName<<" "<<model_pos_x<<" "<<model_pos_y<<endl;	
		moveRobot(0.5,0);
		ros::spinOnce();
		loop_rate.sleep();	
	}
	
	model_pos_x=10.0;
	model_pos_y=10.0;
	stopRobot();
	cout<<"Moving to the middle DONE"<<endl;
}
/*
void DirectionalGraph::moveMiddleTag(){
	ros::Rate loop_rate(10);
	
	while((modelName.find("corridor") !=string::npos || modelName.find("se1") !=string::npos) && modelName==fatherChild[1]  ){
		//cout<<"out of the model: "<< modelName<<endl;
		counter+=1;
		modelName=" ";
		moveRobot(0.5,0);
		ros::spinOnce();
		loop_rate.sleep();
		
	}
	stopRobot();
}*/

int main(int argc, char **argv){
	bool startExploringFlag=true;
	int lOrR=0; //left or right
	int direction[2]={}; //[0]=left(1) of right(2), [1]=East(1), North(2), West(3), South(4)
	int directionTemp[2]={};
	int lastNodeAddedIndex, lastNodeAddedIndexPast;
	string currentModelName,lastNodeSeen;
	int lastEdgeAdded=-1;
	int policyReady=0;
	NodeSemantic tempNode;
	int indexTempNode;

	//ROS
	ros::init(argc, argv, "create_topomap_node");
	ros::NodeHandle n2;
	//ros::Publisher goCorridor = n2.advertise<std_msgs::Int16>("go_corridor", 1);
	//ros::Publisher goDoor = n2.advertise<std_msgs::Int16>("go_cross_door", 1);

	ros::Rate loop_rate(10);


	//Semantic map
	DirectionalGraph topomap;

	topomap.modelName="Other object";
	string globalTarget1="South_se108";
	string globalTarget2="West_se108";
	topomap.stopRobot();	//stop the robot before starting
	while (ros::ok()){

		//Start exploration
		while(startExploringFlag==true){
			topomap.moveRobot(0,-0.4);

			ros::spinOnce();
			loop_rate.sleep();
			cout<<"Searching for first node"<<endl;
			topomap.getDirection(direction);
			
			currentModelName=topomap.modelName;
			if (currentModelName.find("corridor") !=string::npos || currentModelName.find("se1") !=string::npos || currentModelName.find("intersection") !=string::npos){ //find the word corridor or se1 in the modelName and ignore the other objects
				
				if (topomap.adjacencyList2Left.size()==0 && topomap.adjacencyList2Right.size()==0 && currentModelName.find("intersection") ==string::npos){ //0 nodes in the graph
					startExploringFlag=false;
					topomap.stopRobot();
					cout<<"First node found"<<endl;
					topomap.fatherChild[0]=currentModelName;  //for the first node the father and child are the same;
					topomap.fatherChild[1]=currentModelName;  //for the first node the father and child are the same;
					lastNodeAddedIndex=topomap.addNode(currentModelName,direction);
					
					cout<<"First node added go to middle"<<endl;
				
					//move middle of tag 
					topomap.moveMiddleTag();

					lastEdgeAdded=direction[0];
					cout<<"print1"<<endl;
					topomap.printGraph();
				}
				if (currentModelName==globalTarget1 || currentModelName==globalTarget2){
					cout<<"Global target found! "<<currentModelName<<endl;
					return 2; //global target found
				}
			}
		}

		currentModelName=topomap.fatherChild[1];
		if (currentModelName.find("corridor") !=string::npos || currentModelName.find("se1") !=string::npos ){ //find the word corridor or se1 in the modelName and ignore the other objects
			bool newNodeFlag=true;
			for (int i=0; i<topomap.visitedLocations.size();i++){
				if (topomap.visitedLocations[i].name==currentModelName){
					newNodeFlag=false;
				}
			}
			if (newNodeFlag==true) { //if new model found
					
					topomap.getDirection(direction);
					if (direction[0]!=1 && direction[0]!=2)
						cout<<"error  direction"<<endl;
					cout<<"adding new node..."<<endl;
					topomap.addEdge(currentModelName,direction, lastEdgeAdded, lastNodeAddedIndex);
					lastEdgeAdded=direction[0];
					cout<<"addedNode "<<" lastEdgeAdded: "<<lastEdgeAdded<<endl;
					topomap.printGraph();
					
					
			}else{ //node already added
				lastNodeAddedIndex=topomap.findFatherIndex(currentModelName);
				cout<<"addedNode2 "<<lastNodeAddedIndex<<currentModelName<<endl;
			}

			if (currentModelName==globalTarget1 || currentModelName==globalTarget2){
					cout<<"Global target found! "<<currentModelName<<endl;
					return 2; //global target found
			}
		}

		//Creating inital graph with at least 3 nodes
		
		if (topomap.visitedLocations.size()<3){
			directionTemp[0]=direction[0];
			directionTemp[1]=direction[1];
			topomap.getDirection(direction);
			

			tempNode=topomap.visitedLocations[topomap.findFatherIndex(topomap.fatherChild[1])];
			if (topomap.fatherChild[1]==globalTarget1 || topomap.fatherChild[1]==globalTarget2){
					cout<<"Global target found! "<<currentModelName<<endl;
					return 2; //global target found
			}
			topomap.stopRobot();
			ros::spinOnce();
			loop_rate.sleep();
			topomap.goNextEdge(tempNode,directionTemp[1]);
			
		}else{ //There is more than 3 nodes so we can use the CMDP to explore

			cout<<"More than 3 nodes search and expand"<<endl;

			string localTarget=" ";

			//selecting type of  localTarget
			int typelocalTarget=1;
			int maxNumEdges,numEdgesTemp,indexMax, openEdgesCounter;
			map<string, int>::iterator itr;

			cout<<"Picking a target"<<endl;

			switch (typelocalTarget){
				
				case 1:	//pick localTarget randomly
					int pickedLocalTarget,foundOpenEdge, localTargetNotFound;
					cout<<"pick localTarget randomly"<<endl;					
					for (int i=0; i<topomap.visitedLocations.size()*5;i++){ //try topomap.visitedLocations.size()*5 times to find a node with open edges	
						random_device rd; // obtain a random number from hardware
						mt19937 eng(rd()); // seed the generator
						uniform_int_distribution<> distr(0, topomap.visitedLocations.size()-1); // define the range
						cout<<"Paaaa"<<endl;
						foundOpenEdge=0;
						pickedLocalTarget=distr(eng);
						localTarget=topomap.visitedLocations[pickedLocalTarget].name;
						for (itr = topomap.visitedLocations[pickedLocalTarget].visitedEdges.begin(); itr != topomap.visitedLocations[pickedLocalTarget].visitedEdges.end(); ++itr){
							if (itr->second==0){
								foundOpenEdge=1;
								break;
							}else{
								foundOpenEdge=0;
							}
						}
						localTargetNotFound=0;
						if(foundOpenEdge==0){
							cout<<"Couldnt find a localTarget with unvisited edges for node: "<<localTarget<<endl;
							localTargetNotFound= 1;
						}else
							break; //local target found with an open edge
					
					}
					if (localTargetNotFound==1){ //if there is no localTargets with open edges then use any edge from last node 
						localTarget=topomap.visitedLocations[topomap.visitedLocations.size()-2].name;
					}
					break;

				case 2:	//Topological Frontier - pick localTarget  with the higher number of edges with variable TemporalDeadline		
					numEdgesTemp=topomap.visitedLocations[0].visitedEdges.size();
					// maxNumEdges=numEdgesTemp;

					// for (int i=0; i<topomap.visitedLocations.size();i++){
					// 	openEdgesCounter=0;
					// 	for (itr = topomap.visitedLocations[i].visitedEdges.begin(); itr != topomap.visitedLocations[i].visitedEdges.end(); ++itr){
					// 		if (itr->second==0){
					// 			openEdgesCounter+=1;
					// 		}
					// 	}
					// 	if (openEdgesCounter>=maxNumEdges){
					// 		maxNumEdges=openEdgesCounter;
					// 		indexMax=i;
					// 	}
					// }
					// localTarget=topomap.visitedLocations[indexMax].name;
					break;

				case 3:	//Topological Frontier normalize- pick localTarget  with the higher number of edges with variable TemporalDeadline		
					numEdgesTemp=topomap.visitedLocations[0].visitedEdges.size();
					// maxNumEdges=numEdgesTemp;

					// for (int i=0; i<topomap.visitedLocations.size();i++){
					// 	openEdgesCounter=0;
					// 	for (itr = topomap.visitedLocations[i].visitedEdges.begin(); itr != topomap.visitedLocations[i].visitedEdges.end(); ++itr){
					// 		if (itr->second==0){
					// 			openEdgesCounter+=1;
					// 		}
					// 	}
					// 	if (openEdgesCounter>=maxNumEdges){
					// 		maxNumEdges=openEdgesCounter;
					// 		indexMax=i;
					// 	}
					// }
					// localTarget=topomap.visitedLocations[indexMax].name;
					break;
		/*		case 4: //semantic localTarget
				
				if (globalTarget.modelName.find("se1") !=string::npos ){
					vector<int> possibleNodes2;
					for (int j=0; j<topomap.visitedLocations.size();i++){
						openEdgesCounter=0;
						for (itr = topomap.visitedLocations[j].visitedEdges.begin(); itr != topomap.visitedLocations[j].visitedEdges.end(); ++itr){
							if (itr->second==0){
								openEdgesCounter+=1;
								possibleNodes2.push_back(j);
								break; //if at least one edge is open this node i can be a candidate to be visited
							}
						}			
					}
				}
*/
			}

			cout <<"localTarget: "<<localTarget<<endl;
			vector<int> statesCMDP;
			vector<int> actionsCMDP;
			vector<int> nextStatesCMDP;

			int indexNode, tempDeadLine;
			int min, indexMin;
			vector<int> numHops,possibleNodes;
			string line;
			ifstream statesfile("/media/sf_sharedFolderLinuxSemantic/statesPolicy.txt");
			ifstream actionsfile("/media/sf_sharedFolderLinuxSemantic/actionsPolicy.txt");
			ifstream nextStatefile("/media/sf_sharedFolderLinuxSemantic/nextStatesPolicy.txt");
			

			if (topomap.visitedLocations[topomap.visitedLocations.size()-1].name.find(localTarget) ==string::npos && policyReady==0  ){
				
				//Write files to be used by matlab
				ofstream startFile,targetFile, temporalDeadLineFile;
				string s1("/media/sf_sharedFolderLinuxSemantic/");
				string startNameFile=s1+"start.txt";
				string targetNameFile=s1+"target.txt";
				string tempDeadNameFile=s1+"tempDeadLine.txt";
				
				topomap.saveListsToFile();

			
				switch (typelocalTarget){
					
					case 1: //pick localTarget randomly
						startFile.open(startNameFile.c_str());
						targetFile.open(targetNameFile.c_str());
						temporalDeadLineFile.open(tempDeadNameFile.c_str());
						temporalDeadLineFile<<10;
						indexTempNode=topomap.visitedLocations.size()-1; //starting node is the last one added
						cout<<"indexTempNode: "<<indexTempNode<<endl;
						tempNode=topomap.visitedLocations[indexTempNode];
						cout<<"tempNode done"<<endl;
						startFile << indexTempNode+1<<endl;
						targetFile<<topomap.findFatherIndex(localTarget)+1<<endl;
						
						startFile.close();
						targetFile.close();
						temporalDeadLineFile.close();
						cout<<"Start and Local Target saved to be used by matlab."<<endl;
						break;
					case 2: //Topological Frontier - pick localTarget  with the higher number of edges
						// startFile.open(startNameFile.c_str());
						// targetFile.open(targetNameFile.c_str());
						// temporalDeadLineFile.open(tempDeadNameFile.c_str());

						// indexNode=topomap.findFatherIndex(localTarget);
						// temporalDeadLineFile<<50; //in order to find the number of hops I use the CMDP to search the graph
						// //Use cmdp
						// tempNode=topomap.visitedLocations[topomap.visitedLocations.size()-1];
						// startFile << topomap.findFatherIndex( tempNode.name)+1<<endl;
						// targetFile<<topomap.findFatherIndex(localTarget)+1<<endl;
						// startFile.close();
						// targetFile.close();
						// temporalDeadLineFile.close();
						// //wait until the policy is computed and we can read it. if the file endWrinting exist then it means matlab finished writing the policy into files
						// while ( 1 ){
						// 	sleep(2);
						// 	if (isFileExist("/media/sf_sharedFolderLinuxSemantic/endWriting.txt")){
						// 		break;
						// 	}
						// }
						
						// if (statesfile.is_open()){
						// 	while ( getline (statesfile,line) ){
						// 		cout << "state: " <<line << '\n';
						// 		statesCMDP.push_back(stoi(line));
						// 	}
						// 	statesfile.close();
						// }else{
						// 	cout << "Unable to open file states"; 
						// }

						// if (actionsfile.is_open()){
						// 	while ( getline (actionsfile,line) ){
						// 		cout << "action: "<< line << '\n';
						// 		actionsCMDP.push_back(stoi(line));
						// 	}
						// 	actionsfile.close();
						// }else{
						// 	cout << "Unable to open file actions"; 
						// }

						// //delete endWriting File to know it was read correctly

						// if( remove( "/media/sf_sharedFolderLinuxSemantic/endWriting.txt" ) != 0 )
						// 	perror( "Error deleting file" );
						// else
						// 	puts( "File successfully deleted" );

						// //assign temporal deadline depending of number of hops
						// if ( (10+((statesCMDP.size()-1)-5) ) >20)
						// 		tempDeadLine=20;
						// 	else
						// 		tempDeadLine=10+((statesCMDP.size()-1)-5);
						
																
						// startFile.open(startNameFile.c_str());
						// targetFile.open(targetNameFile.c_str());
						// temporalDeadLineFile.open(tempDeadNameFile.c_str());
						// temporalDeadLineFile<<tempDeadLine<<endl;
						// tempNode=topomap.visitedLocations[topomap.visitedLocations.size()-1];
						// startFile << topomap.findFatherIndex( tempNode.name)+1<<endl;
						// targetFile<<topomap.findFatherIndex(localTarget)+1<<endl;
						
						// startFile.close();
						// targetFile.close();
						// temporalDeadLineFile.close();
						break;
					case 3: //Topological Frontier Normalize distance- pick localTarget  with the higher number of edges
						
						// for (int j=0; j<topomap.visitedLocations.size();j++){
						// 	openEdgesCounter=0;
						// 	for (itr = topomap.visitedLocations[j].visitedEdges.begin(); itr != topomap.visitedLocations[j].visitedEdges.end(); ++itr){
						// 		if (itr->second==0){
						// 			openEdgesCounter+=1;
						// 			possibleNodes.push_back(j);
						// 			break; //if at least one edge is open this node i can be a candidate to be visited
						// 		}
						// 	}			
						// }

						// for (int i=0;possibleNodes.size();i++){ //check the number of hops necesary for each possible localTarget
						// 	startFile.open(startNameFile.c_str());
						// 	targetFile.open(targetNameFile.c_str());
						// 	temporalDeadLineFile.open(tempDeadNameFile.c_str());

						// 	temporalDeadLineFile<<50; //in order to find the number of hops I use the CMDP to search the graph
						// 	//Use cmdp
						// 	tempNode=topomap.visitedLocations[topomap.visitedLocations.size()-1];
						// 	startFile << topomap.findFatherIndex( tempNode.name)+1<<endl;
						// 	targetFile<<topomap.findFatherIndex(topomap.visitedLocations[i].name)+1<<endl;
						// 	startFile.close();
						// 	targetFile.close();
						// 	temporalDeadLineFile.close();
						// 	//wait until the policy is computed and we can read it. if the file endWrinting exist then it means matlab finished writing the policy into files
						// 	while ( 1 ){
						// 		sleep(2);
						// 		if (isFileExist("/media/sf_sharedFolderLinuxSemantic/endWriting.txt")){
						// 			break;
						// 		}
						// 	}
							
						// 	if (statesfile.is_open()){
						// 		while ( getline (statesfile,line) ){
						// 			cout << "state: " <<line << '\n';
						// 			statesCMDP.push_back(stoi(line));
						// 		}
						// 		statesfile.close();
						// 	}else{
						// 		cout << "Unable to open file states"; 
						// 	}

						// 	if (actionsfile.is_open()){
						// 		while ( getline (actionsfile,line) ){
						// 			cout << "action: "<< line << '\n';
						// 			actionsCMDP.push_back(stoi(line));
						// 		}
						// 		actionsfile.close();
						// 	}else{
						// 		cout << "Unable to open file actions"; 
						// 	}

						// 	//delete endWriting File to know it was read correctly

						// 	if( remove( "/media/sf_sharedFolderLinuxSemantic/endWriting.txt" ) != 0 )
						// 	perror( "Error deleting file" );
						// 	else
						// 	puts( "File successfully deleted" );

						// 	//assign temporal deadline depending of number of hops
							
						// 	//REVISAR ESTO STATESCMDP TIENE TODOS LOS NODOS NO SOLO EL PATH!!!
						// 	if ( (10+((statesCMDP.size()-1)-5) ) >20)
						// 		tempDeadLine=20;
						// 	else
						// 		tempDeadLine=10+((statesCMDP.size()-1)-5);
							
						// 	numHops.push_back(statesCMDP.size()-1);	
						// }
						
						// startFile.open(startNameFile.c_str());
						// targetFile.open(targetNameFile.c_str());
						// temporalDeadLineFile.open(tempDeadNameFile.c_str());
						// temporalDeadLineFile<<tempDeadLine<<endl;

						// min=numHops[0];
						// for (int k=0;k<numHops.size();k++){	//finding localTarget closest distance
						// 	if (numHops[k]<=min){
						// 		min=numHops[k];
						// 		indexMin=k;
						// 	}
						// }
						// tempNode=topomap.visitedLocations[topomap.visitedLocations.size()-1];
						// startFile << topomap.findFatherIndex( tempNode.name)+1<<endl;
						// targetFile<<topomap.findFatherIndex(topomap.visitedLocations[numHops[indexMin]].name)+1<<endl;
							
						// startFile.close();
						// targetFile.close();
						// temporalDeadLineFile.close();
						break;
				}

				while(1){
					sleep(2); 
					ifstream endWrintingFile("/media/sf_sharedFolderLinuxSemantic/endWriting.txt");
					if (endWrintingFile.is_open()){
						endWrintingFile.close();
						cout<<"listo2"<<endl;
				 		policyReady=1;
						break;
					}else{
						cout << "Unable to open file endWrintingFile"<<endl; 
					}
				}
				
				if (statesfile.is_open()){
					while ( getline (statesfile,line) ){
						//cout << "state: " <<line << '\n';
						statesCMDP.push_back(stoi(line));
					}
					statesfile.close();
				}else{
					cout << "Unable to open file states"<<endl; 
				}
				if (actionsfile.is_open()){
					while ( getline (actionsfile,line) ){
						//cout << "action: "<< line << '\n';
						actionsCMDP.push_back(stoi(line));
					}
					actionsfile.close();
				}else{
					cout << "Unable to open file actions"<<endl; 
				}
				
				if (nextStatefile.is_open()){
					while ( getline (nextStatefile,line) ){
						//cout << "action: "<< line << '\n';
						nextStatesCMDP.push_back(stoi(line));
					}
					nextStatefile.close();
				}else{
					cout << "Unable to open file next states"<<endl; 
				}

				//delete endWriting File to know it was read correctly
							
				if( remove( "/media/sf_sharedFolderLinuxSemantic/endWriting.txt" ) != 0 )
				  perror( "Error deleting file" );
				else
				  puts( "File successfully deleted" );
			}
			
			while( policyReady==1){
					//move robot based on policy
					cout<<"Using Policy"<<endl;
					int indexCurrentState, currentAction,directionCurrentState,nextState,directionNextState,directionToGoNextNode;
						
					int indexFatherTemp=topomap.findFatherIndex(topomap.fatherChild[1]);
					if (indexFatherTemp==-1){
						topomap.printGraph();
						topomap.printVisitedLocations();
						cout<<"Error index couldnt be found for "<<topomap.fatherChild[1]<<endl;
						return -1;
					}
					tempNode=topomap.visitedLocations[indexFatherTemp];	//current node
					
					currentModelName=tempNode.name;
					
					if(currentModelName==localTarget){
						policyReady=0;
						break;
					}
					
					


					topomap.stopRobot();
					for (int i=0; i<statesCMDP.size();i++){
						if (currentModelName.find(topomap.visitedLocations[ statesCMDP[i]-1].name) !=string::npos){  //-1 because when we save the file the list of nodes we start from 1 not from 0
							indexCurrentState=i;
							break;
						}
					}
					currentAction=actionsCMDP[indexCurrentState];
					if (currentAction==0){
						topomap.printGraph();
						topomap.printVisitedLocations();
						cout<<"State without valid action"<<endl;
						return -1;
					}
					//speed  maneuver
					//   1       1       slow stayCenter
					//   1       2       slow crossdoor
					//   3       1       fast stayCenter
					//   3       2       slow crossdoor
					//numberActionsLeft=[1 2 3 4];
					//numberActionsRight=[5 6 7 8];
					topomap.getDirection(direction);
					nextState=nextStatesCMDP[indexCurrentState]-1;
					directionNextState=topomap.visitedLocations[nextState].direction;
					// if (direction[1]!=directionNextState){ //if direction of the picked now is different from the current direction the robot needs to rotate
					// 		topomap.faceNewDirection(directionNextState);
					// 		cout<<"directionCurrentStateAA "<<directionNextState<<endl;
					// }
					// topomap.getDirection(direction);
					// if (direction[0]==topomap.RIGHTNUMBER  && (currentAction==1 || currentAction==2 || currentAction==3 || currentAction==4)){
					// 	topomap.faceNewDirection(topomap.opositeDirection(directionNextState));
					// 	cout<<"directionCurrentStateBB "<<directionNextState<<endl;
					// } 
					// if (direction[0]==topomap.LEFTNUMBER  && (currentAction==5 || currentAction==6 || currentAction==7 || currentAction==8)){
					// 	topomap.faceNewDirection(topomap.opositeDirection(directionNextState));
					// 	cout<<"directionCurrentStateCC "<<directionNextState<<endl;
					// } 
					directionToGoNextNode=-1;
					
					tempNode=topomap.visitedLocations[indexCurrentState];				
					for (itr = topomap.visitedLocations[indexCurrentState].connectedNodes.begin(); itr != topomap.visitedLocations[indexCurrentState].connectedNodes.end(); ++itr){
						if (itr->first==topomap.visitedLocations[nextState].name){
							directionToGoNextNode=itr->second;
						}
					}
				
				
					if (directionToGoNextNode==-1){
						topomap.printGraph();
						topomap.printVisitedLocations();
						cout<<"error obtaining directionToGoNextNode"<<endl;
						return -1; //error
					}
					cout<<"States "<<currentModelName<<" Action "<<currentAction<< " next state: "<<topomap.visitedLocations[nextState].name<<" go to: "<<directionToGoNextNode<<endl;
					//directionCurrentState=tempNode.direction;
					
					// if (direction[1]!=directionCurrentState){ //if direction of the picked now is different from the current direction the robot needs to rotate
					// 		topomap.faceNewDirection(directionCurrentState);
					// 		cout<<"directionCurrentStateAA "<<directionCurrentState<<endl;
					// }
					topomap.getDirection(direction);
					topomap.faceNewDirection(directionToGoNextNode);
					topomap.lastDirectionTaken=directionToGoNextNode; //we save the direction we took to define the direction of new node
					
					//move to middle of tag
					topomap.moveMiddleTag();

					cout<<"in policy middle tag done and lastDirectionTaken: "<<topomap.lastDirectionTaken<<endl;

					std_msgs::Int16 goMsgPolicy;
					topomap.getDirection(direction);
					switch (currentAction){
						case 1:  //slow stayCenter to left
								goMsgPolicy.data=1;
								break;
						case 2: //slow crossdoor to left
								if (direction[1]==1)
									goMsgPolicy.data=1;
								if (direction[1]==2)
									goMsgPolicy.data=2;
								if (direction[1]==3)
									goMsgPolicy.data=3;
								if (direction[1]==4)
									goMsgPolicy.data=4;
								break;
						case 3: //fast stayCenter to left
								goMsgPolicy.data=2;
								break;
						case 4: //fast crossdoor to left
								if (direction[1]==1)
									goMsgPolicy.data=5;
								if (direction[1]==2)
									goMsgPolicy.data=6;
								if (direction[1]==3)
									goMsgPolicy.data=7;
								if (direction[1]==4)
									goMsgPolicy.data=8;
								break;
						case 5: //slow stayCenter right
								goMsgPolicy.data=1;
								break;
						case 6: //slow crossdoor right
								if (direction[1]==1)
									goMsgPolicy.data=1;
								if (direction[1]==2)
									goMsgPolicy.data=2;
								if (direction[1]==3)
									goMsgPolicy.data=3;
								if (direction[1]==4)
									goMsgPolicy.data=4;
								break;
						case 7: //fast stayCenter right
								goMsgPolicy.data=2;
								break;
						case 8: //fast crossdoor right
								if (direction[1]==1)
									goMsgPolicy.data=5;
								if (direction[1]==2)
									goMsgPolicy.data=6;
								if (direction[1]==3)
									goMsgPolicy.data=7;
								if (direction[1]==4)
									goMsgPolicy.data=8;
								break;
					}

					topomap.updateFatherChild=true;
					
					if ((currentAction==1 || currentAction==3 || currentAction==5 || currentAction==7) ){ //go to corridor
						
						cout<<"in policy go corridor"<<endl;
						while(topomap.modelName.find("corridor") ==string::npos ){	
								//cout<<"go corridor 1: "<<topomap.modelName<<endl;
							
								topomap.modelName=" ";
								topomap.goCorridor.publish(goMsgPolicy);
								ros::spinOnce();
								loop_rate.sleep();
								
						}
						tempNode=topomap.visitedLocations[topomap.findFatherIndex(topomap.modelName)];
						topomap.stopRobot();
					}else{
						if (currentAction==2 || currentAction==4 || currentAction==6 || currentAction==8){  //go to door
								cout<<"in policy go door"<<endl;
								
								while((topomap.modelName.find("corridor") ==string::npos && topomap.modelName.find("se1") ==string::npos)  ){
										//cout<<"go door 2: "<<topomap.modelName<<endl;
										
										topomap.modelName=" ";
										topomap.goDoor.publish(goMsgPolicy);
										ros::spinOnce();
										loop_rate.sleep();
										
								}
								tempNode=topomap.visitedLocations[topomap.findFatherIndex(topomap.modelName)];
								topomap.stopRobot();
						}
					}

					if (topomap.modelName.find(localTarget) !=string::npos ){
						policyReady=0;
						
					}
					topomap.updateFatherChild=false;
					//move to middle of tag
					topomap.moveMiddleTag();
					cout<<"in policy middle tag done2"<<endl;

					goMsgPolicy.data=0;
					topomap.goCorridor.publish(goMsgPolicy);
					topomap.goDoor.publish(goMsgPolicy);
					ros::spinOnce();
					loop_rate.sleep();
					topomap.stopRobot(); //localTarget found
					cout<<"in policy stoping robot"<<endl;
					
			}

			// //Keep exploring
			
			if ((topomap.fatherChild[1]!=globalTarget1 || topomap.fatherChild[1]!=globalTarget2)  && policyReady==0){
					tempNode=topomap.visitedLocations[topomap.findFatherIndex(topomap.fatherChild[1])];
					cout<<endl<<"Keep exploring..."<<endl;
					topomap.getDirection(direction);
					try{
						topomap.goNextEdge(tempNode,direction[1]); //send direction of current node
						cout<<"next node reached"<<endl;
					}catch(const std::exception& e){
						cout<<"node no found to select one of  its edges"<<endl;
					}
					topomap.stopRobot();
					ros::spinOnce();
					loop_rate.sleep();
			}else
				if (topomap.fatherChild[1]==globalTarget1 || topomap.fatherChild[1]==globalTarget2){
					cout<<"Global target found! "<<currentModelName<<endl;
					return 2; //global target found
			}

		topomap.stopRobot();
		ros::spinOnce();
		loop_rate.sleep();
		
		}
		
		topomap.stopRobot();
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}

