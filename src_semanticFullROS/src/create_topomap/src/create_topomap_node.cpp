/*This node creates a graph of a oriented topological semantic map. 

Author: Jose Luis Susa Rincon 
September, 2018
Version 1.0
*/

#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "create_topomap_node.hh"
#include <limits>
#include <math.h>
#include <unistd.h>
#include <fstream>
#include <random>
#include "geometry_msgs/Twist.h"
#include <time.h>
#include <pwd.h>
#include "cmdp_solver/solveCMDP.h"

#define _USE_MATH_DEFINES

DirectionalGraph::DirectionalGraph(){

    LEFTNUMBER=1;
    RIGHTNUMBER=2;
    getDirectionManually=0; //0 if we want automatic direction obtained from odom, 1 for manual human direction
	lastDirectionTaken=0;

	fatherChild.push_back(" ");
	fatherChild.push_back(" ");
	updateFatherChild=false;
	childFounded=false;
	saveResultCounter=0;
	model_pos_x=0.0;
	model_pos_y=0.0;

	mydirection_robot_average[0]=-99;
  	mydirection_robot_average[1]=-99;
  	mydirection_robot_average[2]=-99;
  	mydirection_robot_average[3]=-99;
  	mydirection_robot_average[4]=-99;

	collisionFlag=false;

	timeToNewNode=0;

	globalTimeStarts=0;
	globalTimeFinal=0;
	totalTimeToGoal=0;
	timeLeftToGoal=0;

    sub_objects = n_.subscribe("/objectsDetected", 100, &DirectionalGraph::readObjectsDetected, this);
    sub_direction=n_.subscribe("/noisy_compass", 100, &DirectionalGraph::readDirection, this);
	goCorridor = n_.advertise<std_msgs::Int16>("go_corridor", 1);
	goDoor = n_.advertise<std_msgs::Int16>("go_cross_door", 1);

	sub_collision=n_.subscribe("/collision", 100, &DirectionalGraph::collisionDetected, this);
    pub_cmd_vel = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 

	clientSolveCMDP = n_.serviceClient<cmdp_solver::solveCMDP>("solve_cmdp");

}


void DirectionalGraph::collisionDetected(const create_topomap::collision::ConstPtr& msg){
  if (msg->modelName.find("corridor") ==string::npos && msg->modelName.find("se1") ==string::npos && msg->modelName.find("intersection") ==string::npos){
	cout <<"COLLISION!!! I heard: "<<msg->modelName<<endl;
	collisionFlag=true;
	stopRobot();	//stop touching the object of collision
	moveRobot(-0.1,0);
	sleep(1);
	stopRobot();
	saveResultCounter++;
	if(saveResultCounter==1){
		endOfProgram(2); //print fail
	}
	//ros::shutdown();
  }
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

	//cout<<"\n \n print  visitedLocations"<<endl;
	for (int i=0; i<visitedLocations.size();i++){
		cout<<"visitedLocations: "<<visitedLocations[i].name<<endl;
	}

	//cout << "print left graph"<<endl;
	for (vector< vector<string> >::iterator it=adjacencyList2Left.begin(); it !=adjacencyList2Left.end(); ++it){
		myTempvector=*it;
		for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			tempNode=*it2;
			cout <<" "<< tempNode << " #edges: "<<visitedLocations[findFatherIndex(tempNode)].visitedEdges.size();
			/*//Printing the edges
			for (itr = visitedLocations[findFatherIndex(tempNode)].visitedEdges.begin(); itr != visitedLocations[findFatherIndex(tempNode)].visitedEdges.end(); ++itr){
				cout<<""<< itr->first <<"-"<< itr->second<<endl;
			}*/
		}
		cout <<endl;
	}
	//cout << "print right graph"<<endl;
	for (vector< vector<string> >::iterator it=adjacencyList2Right.begin(); it !=adjacencyList2Right.end(); ++it){
		myTempvector=*it;
		for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			tempNode=*it2;
			cout<<" " << tempNode << " #edges: "<<visitedLocations[findFatherIndex(tempNode)].visitedEdges.size();
			/*//Printing the edges
			for (itr = visitedLocations[findFatherIndex(tempNode)].visitedEdges.begin(); itr != visitedLocations[findFatherIndex(tempNode)].visitedEdges.end(); ++itr){
				cout<<""<< itr->first <<"-"<< itr->second<<endl;
			}*/
		}
		cout <<endl;
	}
}
int DirectionalGraph::saveListsToFile(){
	ofstream outfile1,outfile2,outfileMatrixLeft,outfileMatrixRight, outfileListNodes,outfileListNodesType ;
	vector<string>  myTempvector;

	int matrixLeft[visitedLocations.size()][visitedLocations.size()]={};
	int matrixRight[visitedLocations.size()][visitedLocations.size()]={};
	map<string,int> dictionaryNodes;

	string leftFileNameFinal=sharedFolderName+"leftList.txt";
	string rightFileNameFinal=sharedFolderName+"rightList.txt";
	string matrixLeftFileNameFinal=sharedFolderName+"adjMatrixLeft.txt";
	string matrixRightFileNameFinal=sharedFolderName+"adjMatrixRight.txt";
	string listNodesFileNameFinal=sharedFolderName+"listNodes.txt";
	string listNodesTypeFileNameFinal=sharedFolderName+"listNodesType.txt";

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

	//cout << "saving left graph"<<endl;
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
	//cout << "saving right graph"<<endl;
	for(int i=0; i<adjacencyList2Right.size(); i++){
		myTempvector=adjacencyList2Right[i];
		nodeIndexTemp=dictionaryNodes[myTempvector[0]];
		for(int j=1; j<myTempvector.size(); j++){			//we analize the nodes after the first one j=1 and not 0 because of that
			outfile2 << myTempvector[j] << " "; 			//adjacency list
			nodeIndexTemp2=dictionaryNodes[myTempvector[j]]; //adjancency matrix
			matrixRight[nodeIndexTemp][nodeIndexTemp2]=1;	//adjancency matrix
		}
		outfile2 << endl;	//adjacency list
	}

	//printing matrices
	//cout << "saving adjancency matrices"<<endl;

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

	//cout <<"Files Printed"<<endl;

	return 1;
}
int DirectionalGraph::savingFilesForCMDP(float failureProbability,float temporalDeadLine, string localTarget){
	int indexTempNode;
	//For CMDP in Matlab
	ofstream startFile,targetFile, temporalDeadLineFile,failureProbabilityFile;

	string startNameFile=sharedFolderName+"start.txt";
	string targetNameFile=sharedFolderName+"target.txt";
	string tempDeadNameFile=sharedFolderName+"tempDeadLine.txt";
	string tempFailPNameFile=sharedFolderName+"tempFailProb.txt";
	startFile.open(startNameFile.c_str());
	targetFile.open(targetNameFile.c_str());

	temporalDeadLineFile.open(tempDeadNameFile.c_str());
	temporalDeadLineFile<<temporalDeadLine;
	
	failureProbabilityFile.open(tempFailPNameFile.c_str());
	failureProbabilityFile<<failureProbability;

	indexTempNode=findFatherIndex(fatherChild[1]);
	//cout<<"indexTempNode: "<<indexTempNode<<endl;
	
	//we add 1 in order to work with matlab whose indexes start from 1 and not 0 like c++, 
	//so all the indexes sent to matlab are +1, when receiving the indexes from matlab they are -1
	startFile << indexTempNode+1<<endl; 
	
	targetFile<<findFatherIndex(localTarget)+1<<endl;
	startFile.close();
	targetFile.close();
	temporalDeadLineFile.close();
	failureProbabilityFile.close();
	cout<<"Start and Local Target saved to be used by matlab."<<endl;
	return 1;
}

int  DirectionalGraph::addEdge2(string currentModelName, int *direction, int lastEdgeAdded, int lastNodeAddedIndex){
	
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

	//cout<<"lOrR: "<<lOrR<<endl;
	if (lOrR==RIGHTNUMBER){
		//cout<<"right right"<<endl;
		tempDirection[0]=LEFTNUMBER;
		newNodeAddedIndex=addNode2(currentModelName,tempDirection);
		if (newNodeAddedIndex==-1){
			cout<<"ERROR: Error adding edge"<<endl;
			return -1;
		}
		//update visitedEdges
		if (updateVisitedEdges(direction)==-1){
			cout<<"ERROR: Error adding edge"<<endl;
			return -1;
		}
		//cout<<"updateVisitedEdges done"<<endl;
		
	}else{
		if (lOrR==LEFTNUMBER){
			//cout<<"left left"<<endl;
			tempDirection[0]=RIGHTNUMBER;
			newNodeAddedIndex=addNode2(currentModelName,tempDirection);
			if (newNodeAddedIndex==-1){
				cout<<"ERROR: Error adding edge"<<endl;
				return -1;
			}
			//update visitedEdges
			if (updateVisitedEdges(direction)==-1){
				cout<<"ERROR: Error adding edge"<<endl;
				return -1;
			}
			
		}
	}
	
	//printVisitedLocations();
	buildAdjancyMats();
	//printGraph();
	
	return 1;
}

int DirectionalGraph::buildAdjancyMats(){

	map <string, int> :: iterator itr;
	vector<string> tempVector;
	adjacencyList2Left.clear();
	adjacencyList2Right.clear();

	for (int i = 0; i < visitedLocations.size(); i++){
		tempVector.push_back(visitedLocations[i].name);
		adjacencyList2Left.push_back(tempVector);
		adjacencyList2Right.push_back(tempVector);
		tempVector.clear();
	}

	
	for (int i = 0; i < visitedLocations.size(); i++){
		for (itr = visitedLocations[findFatherIndex(visitedLocations[i].name)].connectedNodes.begin(); itr != visitedLocations[findFatherIndex(visitedLocations[i].name)].connectedNodes.end(); ++itr){
			//cout<<""<< itr->first <<"-"<< itr->second<<endl;
			switch (itr->second){
				case 1:
					adjacencyList2Right[i].push_back(itr->first);
					break;
				case 2:
					adjacencyList2Right[i].push_back(itr->first);
					break;
				case 3:
					adjacencyList2Left[i].push_back(itr->first);
					break;
				case 4:
					adjacencyList2Left[i].push_back(itr->first);
					break;
				default:
					cout<<"ERROR: connected node with wrong direction"<<endl;
					return -1;
			}	
		}
	}

	return 1;
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

int DirectionalGraph::getDirectionIntersections(int *direction){

		ros::Rate loop_rate(10);
		direction[0]=0;
		direction[1]=0;
		while (direction[0]==0 && direction[1]==0){
			if (mydirection_robot >=2*M_PI-M_PI/4 || mydirection_robot<=M_PI/4){
				direction[0]=2; //indicates that the new place is on the right of the robot
				direction[1]=1; //indicates that the robot is facing east
				
			}else{
				if (mydirection_robot >M_PI/4 && mydirection_robot<=3*M_PI/4){
					direction[0]=2; //indicates that the new place is on the right of the robot
					direction[1]=2; //indicates that the robot is facing north
					
				}else{
					if (mydirection_robot >3*M_PI/4  && mydirection_robot<=2*M_PI-3*M_PI/4){
						direction[0]=1; //indicates that the new place is on the left of the robot
						direction[1]=3; //indicates that the robot is facing west
						
					}else{
						if (mydirection_robot >2*M_PI-3*M_PI/4 && mydirection_robot<2*M_PI-M_PI/4){
							direction[0]=1; //indicates that the new place is on the left of the robot
							direction[1]=4; //indicates that the robot is facing south
							
						}
					}
				}
			}
			//cout<<"mydirection_robot inter: "<<mydirection_robot<<endl;
			ros::spinOnce();
			loop_rate.sleep();

		}
		if (direction[0]==0 || direction[1]==0){
			cout<<"ERROR: direction is 0 intersections"<<endl;
			return -1;
		}

	return 1;

}

int DirectionalGraph::getDirection(int *direction){

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
				if (direction[0]!=1 && direction[0]!=2){
					cout<<"ERROR: lastDirectionTaken is 0: "<<direction[0]<<endl;
					return -1;
				}
				direction[1]=lastDirectionTaken;
		}else{
			direction[0]=0; 
			direction[1]=0;
			
			if (mydirection_robot >=2*M_PI-M_PI/18 || mydirection_robot<=M_PI/18){
				direction[0]=2; //indicates that the new place is on the right of the robot
				direction[1]=1; //indicates that the robot is facing east
				
			}else{
				if (mydirection_robot >8*M_PI/18 && mydirection_robot<=10*M_PI/18){
					direction[0]=2; //indicates that the new place is on the right of the robot
					direction[1]=2; //indicates that the robot is facing north
					
				}else{
					if (mydirection_robot >17*M_PI/18 && mydirection_robot<=2*M_PI-17*M_PI/18 ){
						direction[0]=1; //indicates that the new place is on the left of the robot
						direction[1]=3; //indicates that the robot is facing west
						
					}else{
						if (mydirection_robot >2*M_PI-10*M_PI/18 && mydirection_robot<2*M_PI-8*M_PI/18){
							direction[0]=1; //indicates that the new place is on the left of the robot
							direction[1]=4; //indicates that the robot is facing south
							
						}
					}
				}
			}
			//cout<<"mydirection_robot: "<<mydirection_robot<<endl;
			if (direction[0]==0 || direction[1]==0 ){
				cout<<"ERROR: direction is 0 getdirection"<<endl;
				return -1;
			}
		}
	}
	//cout<<"direction: "<<direction[0] << " compass: " <<direction[1]<<endl;
	return 1;
}

int DirectionalGraph::addNode2(string currentModelName, int *direction){
	cout<<"Adding a node..."<<endl;
	int lastNodeAddedIndex=-1;
	int lOrR;
	vector<string> tempVector;
	NodeSemantic newNode;
	newNode.name=currentModelName;
	newNode.direction=direction[1];
	newNode.finishedscanningEdgesFlag=false;

	visitedLocations.push_back(newNode);
	if (scanForEdges(direction[1],true,direction[0], currentModelName,visitedLocations.size()-1)<0){
		cout<<"ERROR: scand for edges"<<endl;
		return -1;
	}
	stopRobot();
	lastNodeAddedIndex=visitedLocations.size()-1;

	updateFather(currentModelName);

	return lastNodeAddedIndex;
}

void DirectionalGraph::updateFather(string currentModelName){
	
	if (currentModelName.find("corridor") !=string::npos || currentModelName.find("se1") !=string::npos){ //find the word corridor or se1 in the modelName and ignore the other objects
			//lastFatherIndex=findFatherIndex( currentModelName);
			if (fatherChild[1]!=currentModelName && updateFatherChild==true){ //if current node (child) is already saved dont do anything	
				fatherChild[0]=fatherChild[1];	//father is the previous child
				fatherChild[1]=currentModelName; //child is the new node
				childFounded=true;
				//cout<<"fatherChild: "<<fatherChild[0]<<" "<<fatherChild[1]<<endl;
			}
			//return currentModelName;
	}
}

int DirectionalGraph::scanForEdges(int initialOri,bool equalDirec,int lOrR,string myfirstModelName,int lastNodeAddedIndex){
	cout<<"scanning edges..."<<endl;
	float currentOri;
	int direction[2]={};
	int finalDirection;
	int countDirections;
	//int directionPrecise[2]={};
	ros::Rate loop_rate(10);
	
	//move until it doesnt see corridor or se1 to execute maneuvers
	string outNode=myfirstModelName;

	moveMiddleTag();
	//cout<<"scanning edges middle done"<<endl;

	//rotate until the first orientation changes but still check if there is intersections
	string lastLettersFather,tempFather,lastLettersModelName, currentModelName;
	tempFather=fatherChild[1];
	//lastLettersFather=tempFather[tempFather.size()-3]+tempFather[tempFather.size()-2]+tempFather[tempFather.size()-1]+tempFather[tempFather.size()];
	for (auto it=tempFather.cend()-3; it!=tempFather.cend(); ++it)
		lastLettersFather+= *it;

	countDirections=0;
	while(equalDirec==true){
		if (getDirectionIntersections(direction)==-1){
			cout<<"ERROR: direction intersection 0"<<endl;
			return -1;
		}
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
				 //cout<<"Found a: "<<intersectionsName<<" direction: "<<finalDirection<<" father: "<<tempFather<<endl;				
			}
			if (initialOri-currentOri!=0){
				countDirections+=1;
			}else
				countDirections=0;

			//cout<<"countDirectionsA: "<<countDirections<<endl;
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
		if (getDirectionIntersections(direction)==-1){
			cout<<"ERROR: direction intersection 0"<<endl;
			return -1;
		}
		currentOri=direction[1];
		moveRobot(0,-0.4);
		//cout<<"scanning for edges 2: "<<intersectionsName<<endl;
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
						visitedLocations[lastNodeAddedIndex].finishedscanningEdgesFlag=true;
					}
					 //cout<<"Found b: "<<intersectionsName<<" direction: "<<finalDirection<<" father: "<<tempFather<<endl;				
			}

			if (initialOri-currentOri==0){
				countDirections+=1;
			}else
				countDirections=0;
				
			//cout<<"countDirectionsB: "<<countDirections<<" "<<initialOri<<" "<<currentOri<<endl;
			if (countDirections>20)
				break;
			
			ros::spinOnce();
			loop_rate.sleep();
		}
		intersectionsName=" ";
		ros::spinOnce();
		loop_rate.sleep();

	}
	//locate the robot on the right orientation 
	if (faceNewDirection(initialOri)=="collision"){
			return -2;
	}
	stopRobot();
	ros::spinOnce();
	loop_rate.sleep();
	return 1;
}

void DirectionalGraph::readObjectsDetected(const create_topomap::logicalImage::ConstPtr& msg){
	int compass;
	int direction[2]={};

	if ((msg->modelName.find("corridor") !=string::npos || msg->modelName.find("se1") !=string::npos || msg->modelName.find("intersection") !=string::npos) ){ //find the word corridor or se1 in the modelName and ignore the other objects
		//modelName=msg->modelName+ss.str(); //model name plus compass reference
		//cout<<"modelName: "<<modelName<<endl;
		if (msg->modelName.find("corridor") !=string::npos || msg->modelName.find("se1") !=string::npos){		
			modelName=msg->modelName;
			model_pos_x=msg->pose_pos_x;
			model_pos_y=msg->pose_pos_y;
			updateFather(modelName);		
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
  float directionTemp=msg->data;	//Average the direction to reduice noise
  if (directionTemp<0)
  	directionTemp=2*M_PI+directionTemp;

  if (mydirection_robot_average[0]==-99){
	  mydirection_robot_average[0]=directionTemp;
	  mydirection_robot_average[1]=directionTemp;
	  mydirection_robot_average[2]=directionTemp;
	  mydirection_robot_average[3]=directionTemp;
	  mydirection_robot_average[4]=directionTemp;
  }else{
	  mydirection_robot_average[4]=mydirection_robot_average[3];
	  mydirection_robot_average[3]=mydirection_robot_average[2];
	  mydirection_robot_average[2]=mydirection_robot_average[1];
	  mydirection_robot_average[1]=mydirection_robot_average[0];
	  mydirection_robot_average[0]=directionTemp;
  }
  float sum=mydirection_robot_average[0];
  int counter=1;
  //cout<<"aca: "<<mydirection_robot_average[0];
  for(int i=1;i<4;i++){
	   //cout<<" "<<mydirection_robot_average[i];
	  if (abs(mydirection_robot_average[i]-sum/counter)<M_PI){
	  		sum=sum+mydirection_robot_average[i];
			counter++;
	  }
  }
  mydirection_robot=sum/counter;
  
  //cout<<endl<<mydirection_robot<<" vs "<<directionTemp<<endl;
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
	std_msgs::Int16 goMsg;
	goMsg.data=0;
	goCorridor.publish(goMsg);
	goDoor.publish(goMsg);
	ros::spinOnce();
	loop_rate.sleep();
}

string DirectionalGraph::faceNewDirection(int newDirection){

	cout<<"faceFront: "<<newDirection<<endl;
	ros::Rate loop_rate(10);
	//moveMiddleTag();
	//cout<<"mydirection_robot a: "<<mydirection_robot<<endl;
	if(newDirection==1){ 
		if (mydirection_robot>=0 && mydirection_robot<=M_PI)
			moveRobot(0,-0.4); //CW
		else
			moveRobot(0,0.4);	//CCW
	}else{
		if(newDirection==2){ 
			if (mydirection_robot<=3*M_PI/2 && mydirection_robot>=M_PI/2)
				moveRobot(0,-0.4);
			else
				moveRobot(0,0.4);
		}else{
			if(newDirection==3){ 
				if (mydirection_robot>=0 && mydirection_robot<M_PI)
					moveRobot(0,0.4);
				else
					moveRobot(0,-0.4);
			}else{
				if (mydirection_robot>=M_PI/2 && mydirection_robot<3*M_PI/2)
					moveRobot(0,0.4);
				else
					moveRobot(0,-0.4);
			}
		}
	}
	//cout<<"mydirection_robot b: "<<mydirection_robot<<endl;
	while (1){
		if (collisionFlag==true)
			return "collision";

		if (newDirection==1 && (mydirection_robot >=2*M_PI-M_PI/18 || mydirection_robot<=M_PI/18)){ //east
			stopRobot();
			break;
		}else{
			if (newDirection==2 && (mydirection_robot >8*M_PI/18 && mydirection_robot<=10*M_PI/18)){ //north
				stopRobot();
				break;
				
			}else{
				//west
				if (newDirection==3 && (mydirection_robot >17*M_PI/18 && mydirection_robot<=2*M_PI-17*M_PI/18)){
					stopRobot();
					break;
					
				}else{
					if (newDirection==4 && (mydirection_robot >2*M_PI-10*M_PI/18 && mydirection_robot<2*M_PI-8*M_PI/18)){ //south
						stopRobot();
						break;
					}
				}
			}
		}
		//cout<<"mydirection_robot c: "<<mydirection_robot<<endl;
		ros::spinOnce();
		loop_rate.sleep();
	}
	stopRobot();
	//cout<<"faceFront done"<<endl;
	return "faceFront done";
}

int DirectionalGraph::opositeDirection(int currentDirection){
		//cout<<"faceOff"<<endl;
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
	
	oldNodeIndex=findFatherIndex( fatherChild[0]);
	newNodeIndex=findFatherIndex( fatherChild[1]);
	//cout<<"update visitedEdges here: "<<oldNodeIndex<<" "<<newNodeIndex<<endl;
	if (oldNodeIndex==-1 || newNodeIndex==-1){
		cout<<"Error: finding indexes for father and child"<<endl;
		return -1;
	}
	oldNode=visitedLocations[oldNodeIndex];
	newNode=visitedLocations[newNodeIndex];

	//cout<<endl<<"UPDATING VISITED"<<endl<<endl;

	//updating previous node
	//cout<<"visitedLocations "<<numNodes<<endl;
	
	for (itr = oldNode.edges.begin(); itr != oldNode.edges.end(); ++itr){
		if (itr->second==newNode.direction){
			it = visitedLocations[oldNodeIndex].visitedEdges.find(itr->first); 
			//if (it != oldNode.edges.end())
			it->second = 1;
			visitedLocations[oldNodeIndex].connectedNodes[newNode.name]=itr->second;	
			visitedLocations[oldNodeIndex].times[newNode.name]=timeToNewNode;	
		}
	}
	
	//updating new node

	//if the robot is facing east then it means the edge west is visited because it is comming from the left
	myOpositeDirection=opositeDirection(newNode.direction);

	for (itr = newNode.edges.begin(); itr != newNode.edges.end(); ++itr){
		if (itr->second==myOpositeDirection){

			it = visitedLocations[newNodeIndex].visitedEdges.find(itr->first); 
			//if (it != newNode.edges.end())
			it->second = 1;   
			visitedLocations[newNodeIndex].connectedNodes[oldNode.name]=itr->second;
			visitedLocations[newNodeIndex].times[oldNode.name]=timeToNewNode;	
			timeToNewNode=0; //erase previous value	
			//cout<<"first: "<<itr->first<<" second: "<<it->second<<endl;
		}
	}
	if (newNode.edges.size()==0){
		cout<<"error: no edges"<<endl;
		return -1;
	}
	
 return 1; //no problems
}

//IT chooses for a edge the next direction to go randomly
int DirectionalGraph::goNextEdge(NodeSemantic tempNode, int directionRobot){
	cout<<"Going random Edge"<<endl;
	
	ros::Rate loop_rate(10);
	string newModelName;

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


		if (newDirection!=directionRobot){ //if direction of the picked node is different from the current direction the robot needs to rotate
			if (faceNewDirection(newDirection)=="collision"){
				return -2;
			}
		}
		std_msgs::Int16 goMsg;
		goMsg.data=1;
		string lastNodeSeen=fatherChild[1];
		//cout<<"lastNodeSeen: "<<lastNodeSeen<<" directionRobot: "<<directionRobot<<" newDirection: "<<newDirection<< " new node to go: " << namesEdges[pickedEdge]<<endl;
		//cout<<"Searching new node: "<<lastDirectionTaken<<" - "<<fatherChild[0]<<", "<<fatherChild[1]<<endl;
		//move to middle of tag
		//moveMiddleTag();

		lastDirectionTaken=newDirection; //we save the direction we took to define the direction of new node
		
		
 		if (lastNodeSeen.find("corridor") !=string::npos && newDirection==directionRobot ) { //if robot is pointing
			cout<<"go corridor"<<endl;
			newModelName=goCorridorFunc(lastNodeSeen,1);
			if (newModelName=="collision"){
				return -2;
			}	
			
		}else{
			
			if (lastNodeSeen.find("se1") !=string::npos){
				cout<<"go door a"<<endl;
				newModelName=goCrossDoorFunc(lastNodeSeen,newDirection);
				if (newModelName=="collision"){
					return -2;
				}	
			}
			
			if ( lastNodeSeen.find("corridor") !=string::npos && newDirection!=directionRobot){
				cout<<"go door b"<<endl;
				newModelName=goCrossDoorFunc(lastNodeSeen,newDirection);	
				if (newModelName=="collision"){
					return -2;
				}	
			}
		}
		
		
	}else{
		cout<<"Error Node Without edges"<<endl;
		return -1;
	}

	
	//move to middle of tag
	//moveMiddleTag();

	cout<<"Node reached and lastDirectionTaken: "<<lastDirectionTaken<<" - "<<fatherChild[0]<<", "<<fatherChild[1]<<endl;

	string globalTarget1="South_se108";
	string globalTarget2="West_se108";
	if (newModelName==globalTarget1 || newModelName==globalTarget2){
		cout<<"Global target found! "<<newModelName<<endl;
		endOfProgram(1); //success
		return 2; //global target found
	}

	return 1;
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

int DirectionalGraph::moveMiddleTag(){
	//cout<<"Moving to the middle..."<<endl;
	ros::Rate loop_rate(10);
	float distanceModel=10.0;
	std_msgs::Int16 goMsg;
	goMsg.data=1; //slow
	int counterMiddleBehind=0; //if for some reason the robot doesnt stop because 
	float distanceModelOld=distanceModel;
	while(distanceModel>0.2 ){
		//cout<<"distanceModel: "<< distanceModel<<" "<<modelName<<" "<<model_pos_x<<" "<<model_pos_y<<endl;
		if (distanceModel==distanceModelOld)	//if the distance doesnt change it is because it missed the tag
			counterMiddleBehind++;

		if (counterMiddleBehind>5)	//up to 3 times the same distance, if not the tag was missed
			break;

		//cout<<"out of the model: "<< modelName<<endl;
		distanceModelOld=distanceModel;
		distanceModel=model_pos_x;	
		//cout<<"distanceModel: "<< distanceModel<<" "<<modelName<<" "<<model_pos_x<<" "<<model_pos_y<<endl;	
		goCorridor.publish(goMsg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	model_pos_x=10.0;
	model_pos_y=10.0;
	if(collisionFlag==true){
		cout<<"ERROR: collision going to middle"<<endl;
		return -2;
	}
	stopRobot();
	//cout<<"Moving to the middle DONE"<<endl;
	return 1;
}

string DirectionalGraph::goCorridorFunc(string currentModelName,int valueMessage){
	ros::Rate loop_rate(10);
	string newModelName="";
	updateFatherChild=true; //search for new fathers after being positioned located.
	std_msgs::Int16 goMsg;
	goMsg.data=valueMessage;	
	//int counterOUt=0;
	childFounded=false;
	int startTime, endTime, totalTime;
	startTime = time(NULL);
	while(childFounded==false){	
		//cout<<"go corridor 1: "<<modelName<<endl;
		//counterOUt++;
		goCorridor.publish(goMsg);
		
		if (collisionFlag==true)
			return "collision";
		
		ros::spinOnce();
		loop_rate.sleep();
		//if (counterOUt>1100)
		//			break;
	}

	endTime = time(NULL);
	totalTime = endTime - startTime;
	//cout << "Runtime: " << totalTime << " seconds.";
	timeToNewNode=totalTime;
	globalTimeFinal = globalTimeFinal+timeToNewNode;
	//Expected time - used time. To be used to calculate temporal deadline
	timeLeftToGoal=timeLeftToGoal-timeToNewNode; //I discount the time used to travel until that goal from the global expected time.

	//Saving the times for the slow and fast maneuvers to correlate them with the CMDP time
	if(valueMessage==1)
		slowCorridorTime.push_back(totalTime);
	else if(valueMessage==2)
		fastCorridorTime.push_back(totalTime);

	childFounded=false;
	updateFatherChild=false;
	stopRobot();
	//cout<<"STOP ROBOT go"<<endl;
	
	//move to middle of tag
	if(moveMiddleTag()==-2)
		return "collision";

	return fatherChild[1];
}
string DirectionalGraph::goCrossDoorFunc(string currentModelName,int valueMessage){
	ros::Rate loop_rate(10);
	string newModelName="";
	updateFatherChild=true; //search for new fathers after being positioned located.
	std_msgs::Int16 goMsg;
	goMsg.data=valueMessage;	
	childFounded=false;
	int counterOUt=0;

	int startTime, endTime, totalTime;
	startTime = time(NULL);
	while(childFounded==false){	//cout<<"go door 2: "<<modelName<<endl;
		goDoor.publish(goMsg);
		
		if (collisionFlag==true)
			return "collision";
		
		counterOUt++;		
		if (counterOUt>1000)
				break;

		ros::spinOnce();
		loop_rate.sleep();
	}

	endTime = time(NULL);
	totalTime = endTime - startTime;
	//cout << "Runtime: " << totalTime << " seconds.";
	timeToNewNode=totalTime;
	globalTimeFinal = globalTimeFinal+timeToNewNode;

	//Expected time - used time. To be used to calculate temporal deadline
	timeLeftToGoal=timeLeftToGoal-timeToNewNode; //I discount the time used to travel until that goal from the global expected time.

	//Saving the times for the slow and fast maneuvers to correlate them with the CMDP time
	if(valueMessage<=4)
		slowCrossDoorTime.push_back(totalTime);
	else if(valueMessage>=5)
		fastCrossDoorTime.push_back(totalTime);
		
	childFounded=false;
	updateFatherChild=false;
	stopRobot();
	//cout<<"STOP ROBOT gooooooooooooo"<<endl;
	
	//mov+e to middle of tag
	if(moveMiddleTag()==-2)
		return "collision";

	return fatherChild[1];
}

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int DirectionalGraph::minDistance(int V, int *dist, bool *sptSet){
   // Initialize min value
   int min = INT_MAX, min_index;
  
   for (int v = 0; v < V; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;
  
   return min_index;
}
  
// A utility function to print the constructed distance array
int DirectionalGraph::printSolution(int dist[], int V){
   cout<<"Vertex   Distance from Source"<<endl;
   for (int i = 0; i < V; i++)
      cout<<i<<" "<<dist[i]<<endl;
}
  
// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
int DirectionalGraph::dijkstra(int V, int *graphTemp, int src, int goal, vector<int>  &pathGoalToOri){

	int graph[V][V];
	vector<int> temp;
	vector<vector<int>> parents,parentsTemp;
    int dist[V];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i
  
    bool sptSet[V]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized
  
	//read from the graph from the pointer *graphTemp
	for(int i=0; i<V; ++i){
        for(int j=0; j<V; ++j){
            graph[i][j]= *((graphTemp+i*V) + j);
        }
    }
	//Initialize parent array
	// vector<int> temp;
	for (int i = 0; i < V; i++){
		temp.push_back(i);
		parentsTemp.push_back(temp);
		temp.clear();
	}
    // Initialize all distances as INFINITE and stpSet[] as false
    for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;
  
    // Distance of source vertex from itself is always 0
    dist[src] = 0;
    // Find shortest path for all vertices
	int v;
    for (int count = 0; count < V-1; count++){
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in the first iteration.
       int u = minDistance(V,dist, sptSet);

       // Mark the picked vertex as processed
       sptSet[u] = true;
  
       // Update dist value of the adjacent vertices of the picked vertex.
	   
       for (v = 0; v < V; v++){
  
         // Update dist[v] only if is not in sptSet, there is an edge from 
         // u to v, and total weight of path from src to  v through u is 
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u]+graph[u][v] < dist[v]){
            dist[v] = dist[u] + graph[u][v];
			parentsTemp[v].push_back(u);		
		 }
	   }
	  
    }

	// print the constructed distance array
    //printSolution(dist, V);

	int lastIndexParent,lastVertex;
	pathGoalToOri.push_back(goal);
	
	
	pathGoalToOri.push_back(parentsTemp[goal][1]);
	int counterFailingLastIndexParent=0;
		while(1){
			lastIndexParent=pathGoalToOri[pathGoalToOri.size()-1];
			if(lastIndexParent>100){
				cout<<"ERROR INDEX3 U"<<endl;
				return -1;
			}	
			cout<<"lastIndexParent: "<<lastIndexParent<<endl;
			lastVertex=parentsTemp[lastIndexParent][1];
			
			if (lastIndexParent==src)
				break;
			pathGoalToOri.push_back(lastVertex);

			//if lastIndexParent==src is never reached
			counterFailingLastIndexParent++;
			if(counterFailingLastIndexParent>100)
				return -1;
		}

	return dist[goal];
}

float DirectionalGraph::calcualteTemporalDeadLine(string start, string goal){
	
	int graphMatrix[visitedLocations.size()][visitedLocations.size()];
	int indexRow;
	int nodesToReachGoal=-1;
	vector<int> indexes;
	vector<float> weights;
	map<string, int>::iterator itr;
	NodeSemantic tempNode;
	vector<int> pathGoalToOri;
	float temporalDeadline;

	//create graph in a matrix form
	for(int i=0;i<visitedLocations.size();i++){
		for(int j=0;j<visitedLocations.size();j++){
			graphMatrix[i][j]=0;
		}

	}
	for(int i=0;i<visitedLocations.size();i++){
		tempNode=visitedLocations[0];
		indexRow=-1;
		indexRow=findFatherIndex(visitedLocations[i].name);
		indexes.clear();
		weights.clear();
		for (itr = visitedLocations[i].times.begin(); itr != visitedLocations[i].times.end(); ++itr){
			indexes.push_back(findFatherIndex(itr->first));
			weights.push_back(itr->second);
		}
		for(int j=0;j<indexes.size();j++){
			graphMatrix[indexRow][indexes[j]]=weights[j];
		}	
	}
	//Calculate shortest path
	nodesToReachGoal=dijkstra(visitedLocations.size(),(int *)graphMatrix, findFatherIndex(start),findFatherIndex(goal),pathGoalToOri);
	if (nodesToReachGoal==-1){
		cout<<"ERROR: Finding path to calcualte temporal deadline."<<endl;
		return -1;
	}

	if (timeLeftToGoal>totalTimeToGoal*0.8){
		temporalDeadline=(pathGoalToOri.size()-1) * 36.6746; //the amount of maneuvers to go to the goal is the size of the path -1. if I multiply by the maximum time to reach this goal using the slow maneuvers
	}else if (timeLeftToGoal>totalTimeToGoal*0.5){
		temporalDeadline=(pathGoalToOri.size()-1) * 36.6746/2; 
	}else if (timeLeftToGoal>totalTimeToGoal*0.3){
		temporalDeadline=(pathGoalToOri.size()-1)* 36.6746/3; 
	}
	//cout<<"Calcualte deadline done"<<endl;
	return temporalDeadline;
}

vector<int> DirectionalGraph::getPath2Verx(string start,string goal){
	
	int graphMatrix[visitedLocations.size()][visitedLocations.size()];
	int indexRow;
	int nodesToReachGoal=-1;
	vector<int> indexes;
	vector<float> weights;
	map<string, int>::iterator itr;
	NodeSemantic tempNode;
	vector<int> pathGoalToOri;
	int temporalDeadline;

	if(start!=goal){
		//create graph in a matrix form
		for(int i=0;i<visitedLocations.size();i++){
			for(int j=0;j<visitedLocations.size();j++){
				graphMatrix[i][j]=0;
			}

		}
		for(int i=0;i<visitedLocations.size();i++){
			tempNode=visitedLocations[0];
			indexRow=-1;
			indexRow=findFatherIndex(visitedLocations[i].name);
			indexes.clear();
			weights.clear();
			for (itr = visitedLocations[i].times.begin(); itr != visitedLocations[i].times.end(); ++itr){
				indexes.push_back(findFatherIndex(itr->first));
				weights.push_back(itr->second);
			}
			for(int j=0;j<indexes.size();j++){
				graphMatrix[indexRow][indexes[j]]=weights[j];
			}	
		}

		//Calculate shortest path
		nodesToReachGoal=dijkstra(visitedLocations.size(),(int *)graphMatrix, findFatherIndex(start),findFatherIndex(goal),pathGoalToOri);
		
		if (nodesToReachGoal==-1){
			cout<<"ERROR: Finding path"<<endl;

		}
	}else{
		pathGoalToOri.push_back(findFatherIndex(goal));		
	}
	return pathGoalToOri;
}

//pick any node randomly if it has unvisited edges
string DirectionalGraph::randomExploration(){
	int pickedLocalTarget,foundOpenEdge, localTargetNotFound;
	string localTarget;
	map<string, int>::iterator itr;

	cout<<"pick localTarget randomly"<<endl;					
	for (int i=0; i<visitedLocations.size()*5;i++){ //try visitedLocations.size()*5 times to find a node with open edges	
		random_device rd; // obtain a random number from hardware
		mt19937 eng(rd()); // seed the generator
		uniform_int_distribution<> distr(0, visitedLocations.size()-1); // define the range

		foundOpenEdge=0;
		pickedLocalTarget=distr(eng);
		localTarget=visitedLocations[pickedLocalTarget].name;
		for (itr = visitedLocations[pickedLocalTarget].visitedEdges.begin(); itr != visitedLocations[pickedLocalTarget].visitedEdges.end(); ++itr){
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
		localTarget=visitedLocations[visitedLocations.size()-2].name;
	}

	return localTarget;
}

//pick a node with the highest number of unvisited edges
string DirectionalGraph::topologicalFrontier(){
	string localTarget="noTargetFound";
	int maxNumEdges,indexMax, openEdgesCounter;
	map<string, int>::iterator itr;


	for (int i=0; i<visitedLocations.size();i++){
	 	openEdgesCounter=0;
	 	for (itr = visitedLocations[i].visitedEdges.begin(); itr != visitedLocations[i].visitedEdges.end(); ++itr){
	 		if (itr->second==0){
	 			openEdgesCounter+=1;
	 		}
	 	}
		if(i==0){
			maxNumEdges=openEdgesCounter;
			indexMax=i;
		}else if(openEdgesCounter>=maxNumEdges){
	 		maxNumEdges=openEdgesCounter;
	 		indexMax=i;
		}
	}
	localTarget=visitedLocations[indexMax].name;
	cout<<"topoFrontier target: "<<localTarget<<" maxNumEdges: "<<maxNumEdges<<" indexMax: "<<indexMax<<endl;
	return localTarget;
}

//pick a node that is not very far with unvisited edges
string DirectionalGraph::topologicalFrontierNormalized(string start){
	string localTarget="noTargetFound";
	int maxNumEdges,indexMax, openEdgesCounter;
	int maxDistance,indexMaxDist;
	map<string, int>::iterator itr;
	vector<int> path;
	string goal;
	float gama=0.5;

	vector<float> greedy, dist, finalSValues ;
	//Value of each node S(nodes)=gama*(greedy(nodes)/max(greedy(nodes)))-(1-gama)*(dist(nodes)/max(dist(nodes)))
	//if gama is 0 the greedy part goes to 0, if gama is 1 the distance part goes to 0
	
	//greddy value
	for (int i=0; i<visitedLocations.size();i++){
	 	openEdgesCounter=0;
	 	for (itr = visitedLocations[i].visitedEdges.begin(); itr != visitedLocations[i].visitedEdges.end(); ++itr){
	 		if (itr->second==0){
	 			openEdgesCounter+=1;
	 		}
	 	}
		
		greedy.push_back(openEdgesCounter);

		if(i==0){
			maxNumEdges=openEdgesCounter;
			indexMax=i;
		}else if(openEdgesCounter>=maxNumEdges){
	 		maxNumEdges=openEdgesCounter;
	 		indexMax=i;
		}
	}
	
	
	for(int i=0;i<greedy.size();i++){
		greedy[i]=greedy[i]/maxNumEdges;
	}

	//Distance value
	
	for (int i=0; i<visitedLocations.size();i++){
		goal=visitedLocations[i].name;
		path=getPath2Verx(start, goal);
		dist.push_back(path.size()-1); //getPath2Verx returns the path inclding the starting node so we reduce the size in 1
		if(i==0){
			maxDistance=path.size()-1; //getPath2Verx returns the path inclding the starting node so we reduce the size in 1
			indexMaxDist=i;
		}else if(path.size()-1>=maxDistance){
	 		maxDistance=path.size()-1;
	 		indexMaxDist=i;
		}
		path.clear();
	}

	for(int i=0;i<dist.size();i++){
		dist[i]=dist[i]/maxDistance;
	}
	
	//Calculating final values to select target
	float maxSValue;
	int indexMaxSValue;
	for(int i=0;i<dist.size();i++){
		finalSValues.push_back(gama*greedy[i]-(1-gama)*dist[i]);
		if(i==0){
			maxSValue=finalSValues[i];
			indexMaxSValue=i;
		}else if(finalSValues[i]>=maxSValue){
	 		maxSValue=finalSValues[i];
	 		indexMaxSValue=i;
		}
	}

	localTarget=visitedLocations[indexMaxSValue].name;
	cout<<"Local target found with topological weighted frontier"<<endl;
	return localTarget;
}

//explore corridors first quatrinni
string DirectionalGraph::semanticExploration(string start){
	string localTarget="noTargetFound";
	map<string, int>::iterator itr;
	int openEdgesCounter;
	vector<int> path;
	string goal;
	vector<float>  dist ;

	if ( start.find("corridor") !=string::npos){ //if start is a corridor and has unvisited edges use start as localTarget to keep exploring it
		openEdgesCounter=0;
	 	for (itr = visitedLocations[findFatherIndex(start)].visitedEdges.begin(); itr != visitedLocations[findFatherIndex(start)].visitedEdges.end(); ++itr){
	 		if (itr->second==0){
	 			openEdgesCounter+=1;
	 		}
	 	}
		if (openEdgesCounter>0)
			localTarget= start;	
	}
	if(start.find("corridor") ==string::npos || localTarget=="noTargetFound"){ //find the closest node that is a corridor 

		//Distance value
		int minDistance,indexMinDist;
		bool foundMin=false;
		for (int i=0; i<visitedLocations.size();i++){
			goal=visitedLocations[i].name;
			path=getPath2Verx(start, goal);
			dist.push_back(path.size()-1); //getPath2Verx returns the path inclding the starging node so we reduce the size in 1
			if(foundMin==false && goal.find("corridor") !=string::npos){
				minDistance=path.size()-1; //getPath2Verx returns the path inclding the starging node so we reduce the size in 1
				indexMinDist=i;
				foundMin=true;
			}else if(foundMin==true && path.size()-1<=minDistance && goal.find("corridor") !=string::npos){
				minDistance=path.size()-1;
				indexMinDist=i;
			}
			path.clear();
		}

		localTarget= visitedLocations[indexMinDist].name;
		
	}

	
	return localTarget;

}

//explore corridors first but explore the closest room first before keep exploraing the corridor
string DirectionalGraph::semanticExplorationRooms(string start){
	string localTarget="noTargetFound";

	
	int openEdgesCounter;
	vector<int> path;
	string goal;
	vector<float>  dist ;
	std::map<int, string> orderedNodesByDist;
	bool findClorsestCorridor=false;

	//Distance value
	if ( start.find("corridor") !=string::npos){ //if start is a corridor and has unvisited edges use start as localTarget to keep exploring it
	 	for (map<string, int>::iterator  itr = visitedLocations[findFatherIndex(start)].visitedEdges.begin(); itr != visitedLocations[findFatherIndex(start)].visitedEdges.end(); ++itr){
	 		if (itr->second==0){ //if there is an open edge
			 	//if the open edge is perpendicular to the direction of the corridor then it means that most likely there is a room there so it should visit it first
	 			if(visitedLocations[findFatherIndex(start)].edges[itr->first]!=visitedLocations[findFatherIndex(start)].direction || visitedLocations[findFatherIndex(start)].edges[itr->first]!=opositeDirection(visitedLocations[findFatherIndex(start)].direction)){
					localTarget= start;
				}
	 		}
	 	}	
		 //if  the corridor doesnt have unvisited edges perpendicular to the corridor then find closest corridor with open edges 
		 findClorsestCorridor=true; 
	}else{ //find the closest node that is a corridor 
		findClorsestCorridor=true; 
	}

	if (findClorsestCorridor==true){
		//Distances to nodes
		for (int i=0; i<visitedLocations.size();i++){
			goal=visitedLocations[i].name;
			path=getPath2Verx(start, goal);
			dist.push_back(path.size()-1); //getPath2Verx returns the path inclding the starging node so we reduce the size in 1
			orderedNodesByDist[path.size()-1]=goal; //ordered map with the distance as the key, by default the map will be ordered if using a iterator
			path.clear();
		}
		//from the closest node check if there is one with a open edge that is perpendicular to the direction of the corridor
		string closestNodeOpenEdge="noTargetFound";
		bool foundOpenEdge=false;
		for(map<int, string>::iterator itr2=orderedNodesByDist.begin(); itr2!=orderedNodesByDist.end();itr2++){
			if ( itr2->second.find("corridor") !=string::npos){ //if itr2->second is a corridor and has unvisited edges use itr2->second as localTarget to keep exploring it
				for (map<string, int>::iterator  itr = visitedLocations[findFatherIndex(itr2->second)].visitedEdges.begin(); itr != visitedLocations[findFatherIndex(itr2->second)].visitedEdges.end(); ++itr){
					if (itr->second==0){ //if there is an open edge
						//if the open edge is perpendicular to the direction of the corridor then it means that most likely there is a room there so it should visit it first
						if(visitedLocations[findFatherIndex(itr2->second)].edges[itr->first]!=visitedLocations[findFatherIndex(itr2->second)].direction || visitedLocations[findFatherIndex(itr2->second)].edges[itr->first]!=opositeDirection(visitedLocations[findFatherIndex(itr2->second)].direction)){
							localTarget= itr2->second;
						}
						if(foundOpenEdge==false){ // the closest node with and open edge, use it if there are not perpedicular open edges
							closestNodeOpenEdge=itr2->second;
							foundOpenEdge=true;
						}
					}
				}	
			}
		}
		//if after checking all nodes noone has a perpendicular open edge then go to the closest one with and open edge
		localTarget=closestNodeOpenEdge;
	}
	
	
	return localTarget;

}


//This function saves the files with the data at the end of the program if there was a success of failure
void DirectionalGraph::endOfProgram(int succFailFlag){
	cout<<"End of the program!..."<<succFailFlag<<endl;
	//saving the data experiment to a file
	std::fstream fs,fsTimesSlowCorr,fsTimesFastCorr,fsTimesSlowDoor,fsTimesFastDoor,fsGraph;
	string dataExpFileName=sharedFolderName+"dataExperimentsExploration"+".txt";
	string dataTimesSlowCorrFileName=sharedFolderName+"dataTimesSlowCorr.txt";
	string dataTimesSlowDoorFileName=sharedFolderName+"dataTimesSlowDoor.txt";
	string dataTimesFastCorrFileName=sharedFolderName+"dataTimesFastCorr.txt";
	string dataTimesFastDoorFileName=sharedFolderName+"dataTimesFastDoor.txt";
	//string dataGraphFileName=sharedFolderName+"dataGraphInfo.txt";
cout<<"dataExpFileName: "<< dataExpFileName<<endl;
	fs.open (dataExpFileName, std::fstream::out | std::fstream::app);
	
	fsTimesSlowCorr.open (dataTimesSlowCorrFileName, std::fstream::out | std::fstream::app);
	fsTimesSlowDoor.open (dataTimesSlowDoorFileName, std::fstream::out | std::fstream::app);
	fsTimesFastCorr.open (dataTimesFastCorrFileName, std::fstream::out | std::fstream::app);
	fsTimesFastDoor.open (dataTimesFastDoorFileName, std::fstream::out | std::fstream::app);
	//fsGraph.open (dataGraphFileName, std::fstream::out | std::fstream::app);

	//saving the size of the graph and the total of open edges
	int counterOpenEdges=0;
	int counterVisitedEdges=0;
	for(int i=0;i<visitedLocations.size();i++){
		for (map<string, int>::iterator itr = visitedLocations[i].visitedEdges.begin(); itr != visitedLocations[i].visitedEdges.end(); ++itr){
	 		if (itr->second==0){
	 			counterOpenEdges++;
	 		}else{
				 counterVisitedEdges++;
			 }
	 	}
	}

	//fsGraph<<visitedLocations.size()<<" "<<counterOpenEdges<<" "<<counterVisitedEdges<<endl; //number of nodes and number of unvisited and visited edges


	//if succFailFlag==1 success, 2 for crashing, 3 for deadline reached and 4 for no policy found
	if(succFailFlag==1){
		fs << "1"<<" "<<globalTimeFinal-globalTimeStarts<<" "<<totalTimeToGoal<<" "<<timeLeftToGoal<<" "<<1<<" "<<visitedLocations.size()<<" "<<counterOpenEdges<<" "<<counterVisitedEdges<<endl;
	}else if(succFailFlag==2){
		fs << "0"<<" "<<globalTimeFinal-globalTimeStarts<<" "<<totalTimeToGoal<<" "<<timeLeftToGoal<<" "<<2<<" "<<visitedLocations.size()<<" "<<counterOpenEdges<<" "<<counterVisitedEdges<<endl;
	}else if(succFailFlag==3){
		fs << "0"<<" "<<globalTimeFinal-globalTimeStarts<<" "<<totalTimeToGoal<<" "<<timeLeftToGoal<<" "<<3<<" "<<visitedLocations.size()<<" "<<counterOpenEdges<<" "<<counterVisitedEdges<<endl;
	}else if(succFailFlag==4){
		fs << "0"<<" "<<globalTimeFinal-globalTimeStarts<<" "<<totalTimeToGoal<<" "<<timeLeftToGoal<<" "<<4<<" "<<visitedLocations.size()<<" "<<counterOpenEdges<<" "<<counterVisitedEdges<<endl;			
	}

	//saving the times for each maneuver to measure its time and find the relation with matlab and the CMDP time
	for(int i=0;i<slowCorridorTime.size();i++){
		fsTimesSlowCorr<<slowCorridorTime[i]<<endl;
		
	}
	for(int i=0;i<slowCrossDoorTime.size();i++){
		fsTimesSlowDoor<<slowCrossDoorTime[i]<<endl;
		
	}
	for(int i=0;i<fastCorridorTime.size();i++){
		fsTimesFastCorr<<fastCorridorTime[i]<<endl;
		
	}
	for(int i=0;i<fastCrossDoorTime.size();i++){
		fsTimesFastDoor<<fastCrossDoorTime[i]<<endl;
		
	}

	
	fs.close();
	fsTimesSlowCorr.close();
	fsTimesSlowDoor.close();
	fsTimesFastCorr.close();
	fsTimesFastDoor.close();
	//fsGraph.close();
}

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
	map<string, int>::iterator itr;

	//ROS
	ros::init(argc, argv, "create_topomap_node");
	ros::NodeHandle n2;
	//ros::Publisher goCorridor = n2.advertise<std_msgs::Int16>("go_corridor", 1);
	//ros::Publisher goDoor = n2.advertise<std_msgs::Int16>("go_cross_door", 1);

	ros::Rate loop_rate(10);


	//Semantic map
	DirectionalGraph topomap;

	//service client to solve cmdp
	cmdp_solver::solveCMDP srvSolveCmdp;
	srvSolveCmdp.request.startSolver=true;


	topomap.modelName="Other object";

	 //get the home path if we want to save the files to the home folder
	const char *homedir;
	homedir=getenv("HOME");
	if ( homedir == NULL ) {
    	homedir = getpwuid(getuid())->pw_dir;
	}	
    string s1(homedir);
	string setupFileName=s1+"/sharedFolderLinuxSemantic/setupTest.txt";
	//setupFileName="/media/sharedFolderLinuxSemantic/setupTest.txt"; //you can define your own folder to share the files

	//Read Configuration File **********
	//This file contains the globalTarget1,globalTarget2,timeToGoal,failureProb,typelocalTarget
	ifstream setupTest;
	setupTest.open (setupFileName, std::ifstream::in);
	
	//CMDP
	string globalTarget1;//="South_se108";
	string globalTarget2;//="West_se108";
	float failureProb;//0.01

	//*** IMPORTANT about the temporalDeadLine****
	//This needs to be calculated every time we call the CMDP. 
	//Because the exploration always chooses the slowest maneuvers, calculating the number of manuevers used to go from 
	//point A to B we can calcualte the max time spent, if we want to go faster we reduce the time. 

	float temporalDeadLine;

	//selecting type of  localTarget
	int typelocalTarget;//=1; //1: random exploration, 2: topological frontier

	//Shared Folder to comunicate with Matlab
	string tempName; //save the name of the different files using the sharedFolderName

	string lineToRead;
	int counterSetupVariables=1;
	if (setupTest.good()) {
		while ( getline (setupTest,lineToRead) ){
			//cout << "state: " <<line << '\n';
			switch (counterSetupVariables){
				case 1:
					globalTarget1=lineToRead;
					break;
				case 2:
					globalTarget2=lineToRead;
					break;
				case 3:
					topomap.totalTimeToGoal=stoi(lineToRead)*1.1; //The average time to reach the goal + 10% extra
					topomap.timeLeftToGoal=topomap.totalTimeToGoal; //The average time to reach the goal + 10% extra
					break;
				case 4:
					failureProb=atof(lineToRead.c_str());
					break;
				case 5:
					typelocalTarget=stoi(lineToRead);
					break;
				case 6:
					topomap.sharedFolderName=lineToRead;
					break;
			}		
			counterSetupVariables++;
		}
	}else{
		cout<<"ERROR: couldnt read the configuration file"<<endl;
		return -1;
	}

	//Files from Matlab
	ifstream endWrintingFile,statesfile,actionsfile,nextStatefile;

	//topomap.globalTimeStarts = time(NULL);

	topomap.stopRobot();	//stop the robot before starting
	while (ros::ok()){


		//Start exploration
		while(startExploringFlag==true){
			topomap.moveRobot(0,-0.4);

			ros::spinOnce();
			loop_rate.sleep();
			//cout<<"Searching for first node"<<endl;

			while (topomap.getDirectionIntersections(direction)==-1){
				cout<<"ERROR: direction first node"<<endl;
				return -1;
			}
			topomap.lastDirectionTaken=direction[1];
			currentModelName=topomap.modelName;
			if (currentModelName.find("corridor") !=string::npos || currentModelName.find("se1") !=string::npos || currentModelName.find("intersection") !=string::npos){ //find the word corridor or se1 in the modelName and ignore the other objects
				
				if (topomap.adjacencyList2Left.size()==0 && topomap.adjacencyList2Right.size()==0 && currentModelName.find("intersection") ==string::npos){ //0 nodes in the graph
					startExploringFlag=false;
					topomap.stopRobot();
					//cout<<"First node found"<<endl;
					topomap.fatherChild[0]=" ";  //for the first node the father and child are the same;
					topomap.fatherChild[1]=currentModelName;  //for the first node the father and child are the same;
					lastNodeAddedIndex=topomap.addNode2(currentModelName,direction);
					if (lastNodeAddedIndex==-1){
						cout<<"ERROR: Error adding first node"<<endl;
						return -1;
					}
					//cout<<"First node added "<<endl;
				
					//move middle of tag 
					//topomap.moveMiddleTag();

					lastEdgeAdded=direction[0];
					topomap.printGraph();
				}
				if (currentModelName==globalTarget1 || currentModelName==globalTarget2){
					cout<<"Global target found! "<<currentModelName<<endl;
					topomap.endOfProgram(1); //success
					return 2; //global target found
				}
			}
		}

		//when the time left to reach the goal is <0 that means failure.
		//cout<<" topomap.timeLeftToGoal***************************: "<<time(NULL)<<" "<<topomap.globalTimeStarts<<" "<<time(NULL)-topomap.globalTimeStarts<<" "<<topomap.timeLeftToGoal<<endl;
		if ( topomap.timeLeftToGoal<0){	//if a policy 
			cout<<"FAILURE: time to reach goal is exceded: "<<topomap.timeLeftToGoal<<endl;
			topomap.endOfProgram(3); //fail
			return -1;
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
					
					if (topomap.getDirection(direction)==-1){
						cout<<"ERROR: direction new node"<<endl;
						return -1;
					}

					cout<<"adding new node..."<<endl;
					if (topomap.addEdge2(currentModelName,direction, lastEdgeAdded, lastNodeAddedIndex)==-1){
							cout<<"ERROR: Error adding edge"<<endl;
							return -1;
					}
					
					lastEdgeAdded=direction[0];
					//cout<<"addedNode "<<" lastEdgeAdded: "<<lastEdgeAdded<<endl;
						
			}else{ //node already added
				lastNodeAddedIndex=topomap.findFatherIndex(currentModelName);
				topomap.moveMiddleTag();
				cout<<"node already added "<<lastNodeAddedIndex<<currentModelName<<endl;
			}		
		}

		//Creating inital graph with at least 3 nodes
		if (topomap.visitedLocations.size()<3){
			directionTemp[0]=direction[0];
			directionTemp[1]=direction[1];
			if (topomap.getDirection(direction)==-1){
				cout<<"ERROR: direction 3 nodes"<<endl;
				return -1;
			}

			tempNode=topomap.visitedLocations[topomap.findFatherIndex(topomap.fatherChild[1])];
			
			topomap.stopRobot();
			ros::spinOnce();
			loop_rate.sleep();
			int resultNextEdge=topomap.goNextEdge(tempNode,directionTemp[1]);
			switch (resultNextEdge){
				case 1:
					cout<<"next node reached"<<endl;
					break;
				case -1:
					cout<<"ERROR: goNextEdge "<<endl;
					return -1;
					break;
				case 2:
					cout<<"AWESOME!! goal found b! "<<endl;
					return 2;
					break;
				case -2:
					cout<<"collision: goNextEdge"<<endl;	//collision exit
					return -2;
					break;
			}
			
		}else{ //There is more than 3 nodes so we can use the CMDP to navigate

			cout<<"More than 3 nodes search and expand"<<endl;
			//move middle of tag 
			topomap.moveMiddleTag();
			
			vector<int> statesCMDP;
			vector<int> actionsCMDP;
			vector<int> nextStatesCMDP;

			int indexNode, tempDeadLine;
			int min, indexMin;
			vector<int> numHops,possibleNodes;
			string line;
			string localTarget=" ";

			if (topomap.visitedLocations[topomap.visitedLocations.size()-1].name.find(localTarget) ==string::npos && policyReady==0  ){
				//saving graph for CMDP
				if (topomap.saveListsToFile()!= 1)	{
					cout<<"ERROR: files could not be saved"<<endl;
					return -1;
				}
				
				cout<<"Picking a target"<<endl;
				//cout<<"topomap.fatherChild[1]: "<< topomap.fatherChild[1]<<endl;
				string start=topomap.visitedLocations[topomap.findFatherIndex(topomap.fatherChild[1])].name;
				//cout<<"start:  "<< start<<endl;
				switch (typelocalTarget){			
					case 1:	//pick localTarget randomly
							localTarget=topomap.randomExploration();
							if(start!=localTarget){
								temporalDeadLine=topomap.calcualteTemporalDeadLine(start,localTarget);
								if (temporalDeadLine==-1){
									cout<<"ERROR: could not find a path"<<endl;
									return -1;
								}
							}
							
							break;
					case 2:	//Topological Frontier - pick localTarget  with the higher number of edges with variable TemporalDeadline		
							localTarget=topomap.topologicalFrontier();
							if(start!=localTarget){
								temporalDeadLine=topomap.calcualteTemporalDeadLine(start,localTarget);
								if (temporalDeadLine==-1){
									cout<<"ERROR: could not find a path"<<endl;
									return -1;
								}
							}
							break;
					case 3:	//Topological Frontier normalize- pick localTarget  with the higher number of edges with variable TemporalDeadline		
							localTarget=topomap.topologicalFrontierNormalized(start);
							if(start!=localTarget){
								temporalDeadLine=topomap.calcualteTemporalDeadLine(start,localTarget);
								if (temporalDeadLine==-1){
									cout<<"ERROR: could not find a path"<<endl;
									return -1;
								}
							}
							break;
					case 4: //semantic localTarget
							localTarget=topomap.semanticExploration( start);
							if (localTarget=="noTargetFound"){
								cout<<"Error: no target could be found"<<endl;
								return -1;
							}
							if(start!=localTarget){
								temporalDeadLine=topomap.calcualteTemporalDeadLine(start,localTarget);
								if (temporalDeadLine==-1){
									cout<<"ERROR: could not find a path"<<endl;
									return -1;
								}
							}
							break;
					case 5: //semantic localTarget
							localTarget=topomap.semanticExplorationRooms( start);
							if (localTarget=="noTargetFound"){
								cout<<"Error: no target could be found"<<endl;
								return -1;
							}
							if(start!=localTarget){
								temporalDeadLine=topomap.calcualteTemporalDeadLine(start,localTarget);
								if (temporalDeadLine==-1){
									cout<<"ERROR: could not find a path"<<endl;
									return -1;
								}
							}
							break;
				}
				cout <<"localTarget: "<<localTarget<<endl;

				if (start!=localTarget){

					//Run CMDP
					//topomap.savingFilesForCMDP(failureProb,temporalDeadLine,localTarget);
	
					srvSolveCmdp.request.start=topomap.findFatherIndex(topomap.fatherChild[1])+1;
					srvSolveCmdp.request.target=topomap.findFatherIndex(localTarget)+1;
					srvSolveCmdp.request.tempDeadLine=temporalDeadLine;
					srvSolveCmdp.request.failProb=failureProb;

					cout<<"Sent service: "<< (int)srvSolveCmdp.request.start<<" "<<(int)srvSolveCmdp.request.target<<" "<<srvSolveCmdp.request.tempDeadLine<<" "<<srvSolveCmdp.request.failProb<<endl;
						if (topomap.clientSolveCMDP.call(srvSolveCmdp)){
							int response=srvSolveCmdp.response.flagFinish;
							cout<<"Response: "<<response<<endl;
							if(response==1){
								policyReady=1;
								sleep(2); 
							}else if(response==-1){
								cout<<"FAILURE: Policy could not be found: "<<topomap.timeLeftToGoal<<endl;
								topomap.endOfProgram(4); //fail
								return -1;
							}else{
								cout<<"ERROR: Building the matrix for solver."<<endl;
								return -1;
							}
						}else{
							cout<<"ERROR: Failed to call service solve cmdp"<<endl;
							return -1;
						}

					int counterOut=0;
					/*
					while(1){
						sleep(2); 
						tempName=topomap.sharedFolderName+"endWriting.txt";
						endWrintingFile.open (tempName, std::ifstream::in);

						if (endWrintingFile.good()) {
							if('1' == endWrintingFile.get()){
								policyReady=1;
								break;
							}else{
								cout << "Unable to open file endWrintingFile: "<<counterOut<<endl; 
								counterOut++;
								if (counterOut>40){
									cout<<"ERROR a: endWrintingFile couldnt be read"<<endl;
									return -1;
								}
							}
						}else{
							cout << "Unable to open file endWrintingFile: "<<counterOut<<endl; 
							counterOut++;
							if (counterOut>40){
								cout<<"ERROR b: endWrintingFile couldnt be read"<<endl;
								return -1;
							}
						}
						endWrintingFile.close();
	
					}
					*/
					counterOut=0;
					tempName=topomap.sharedFolderName+"statesPolicy.txt";
					statesfile.open (tempName, std::ifstream::in);
					if (statesfile.good()) {
						while ( getline (statesfile,line) ){
							//cout << "state: " <<line << '\n';
							statesCMDP.push_back(stoi(line));
						}
						
					}else{
						cout << "Unable to open file statesfile: "<<counterOut<< " "<< tempName<<endl; 
						counterOut++;
						if (counterOut>40){
							cout<<"ERROR: statesfile couldnt be read"<<endl;
							return -1;
						}
					}
					statesfile.close();

					
					counterOut=0;
					tempName=topomap.sharedFolderName+"actionsPolicy.txt";
					actionsfile.open (tempName, std::ifstream::in);
					if (actionsfile.good()) {
						while ( getline (actionsfile,line) ){
							//cout << "action: "<< line << '\n';
							actionsCMDP.push_back(stoi(line));
						}
					}else{
						cout << "Unable to open file actionsfile: "<<counterOut<<endl; 
						counterOut++;
						if (counterOut>40){
							cout<<"ERROR: actionsfile couldnt be read"<<endl;
							return -1;
						}
					}
					actionsfile.close();

					counterOut=0;
					tempName=topomap.sharedFolderName+"nextStatesPolicy.txt";
					nextStatefile.open (tempName, std::ifstream::in);
					if (nextStatefile.good()) {
						while ( getline (nextStatefile,line) ){
							//cout << "action: "<< line << '\n';
							nextStatesCMDP.push_back(stoi(line));
						}
						//cout<<"nextStatefile done"<<endl;
					}else{
						cout << "Unable to open file nextStatefile: "<<counterOut<<endl; 
						counterOut++;
						if (counterOut>40){
							cout<<"ERROR: nextStatefile couldnt be read"<<endl;
							return -1;
						}
					}
					nextStatefile.close();			  
				}
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

					indexCurrentState=indexFatherTemp;

					currentAction=actionsCMDP[indexCurrentState];
					
					//When policy is 0 means that the was an error with the CMDP and most likely one node had 0 edges so there wasnt a policy for that node.

					if (currentAction==0){	//if a policy 
						topomap.printGraph();
						topomap.printVisitedLocations();
						cout<<"ERROR: State without valid action"<<endl;
						return -1;
					}

					//When policy is 0 for the states means that the CMDP couldnt be solved and that means failure.
					if (currentAction==-1){	//if a policy wasnt found then all the actions are -1 and the next states too
						cout<<"FAILURE: Policy could not be found: "<<topomap.timeLeftToGoal<<endl;
						topomap.endOfProgram(4); //fail
						return -1;
					}

					//speed  maneuver
					//   0       0       fast stayCenter
    				//   0       1       fast crossdoor
					//   2       0       slow stayCenter
					//   2       1       slow crossdoor
					//numberActionsLeft=[1 2 3 4];
					//numberActionsRight=[5 6 7 8];
					if (topomap.getDirection(direction)==-1){
						cout<<"ERROR: direction policy 1"<<endl;
						return -1;
					}
					nextState=nextStatesCMDP[indexCurrentState]-1;
					directionNextState=topomap.visitedLocations[nextState].direction;
				
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
						cout<<"ERROR: Obtaining directionToGoNextNode"<<endl;
						return -1; //error
					}
					cout<<"States "<<currentModelName<<" Action "<<currentAction<< " next state: "<<topomap.visitedLocations[nextState].name<<" go to: "<<directionToGoNextNode<<endl;

					if (topomap.getDirection(direction)==-1){
						cout<<"ERROR: direction policy 2"<<endl;
						return -1;
					}
				
					if (topomap.faceNewDirection(directionToGoNextNode)=="collision"){
						return -2;
					}
					topomap.lastDirectionTaken=directionToGoNextNode; //we save the direction we took to define the direction of new node

					cout<<"in policy middle tag done and lastDirectionTaken: "<<topomap.lastDirectionTaken<<endl;

					std_msgs::Int16 goMsgPolicy;
					if (topomap.getDirection(direction)==-1){

						cout<<"ERROR: direction policy 3"<<endl;
						return -1;
					}
					switch (currentAction){
						case 3:  //slow stayCenter to left
								goMsgPolicy.data=1;
								break;
						case 4: //slow crossdoor to left
								if (direction[1]==1)
									goMsgPolicy.data=1;
								if (direction[1]==2)
									goMsgPolicy.data=2;
								if (direction[1]==3)
									goMsgPolicy.data=3;
								if (direction[1]==4)
									goMsgPolicy.data=4;
								break;
						case 1: //fast stayCenter to left
								goMsgPolicy.data=2;
								break;
						case 2: //fast crossdoor to left
								if (direction[1]==1)
									goMsgPolicy.data=5;
								if (direction[1]==2)
									goMsgPolicy.data=6;
								if (direction[1]==3)
									goMsgPolicy.data=7;
								if (direction[1]==4)
									goMsgPolicy.data=8;
								break;
						case 7: //slow stayCenter right
								goMsgPolicy.data=1;
								break;
						case 8: //slow crossdoor right
								if (direction[1]==1)
									goMsgPolicy.data=1;
								if (direction[1]==2)
									goMsgPolicy.data=2;
								if (direction[1]==3)
									goMsgPolicy.data=3;
								if (direction[1]==4)
									goMsgPolicy.data=4;
								break;
						case 5: //fast stayCenter right
								goMsgPolicy.data=2;
								break;
						case 6: //fast crossdoor right
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
		
					string newModelName="";
					cout<<"Action Chosen!!!!!  "<<goMsgPolicy.data<<endl;
					if ((currentAction==1 || currentAction==3 || currentAction==5 || currentAction==7) ){ //go to corridor
						
						cout<<"in policy go corridor"<<endl;
						newModelName=topomap.goCorridorFunc(currentModelName,goMsgPolicy.data);	
						if (newModelName=="collision"){
							return -1;
						}		
						
					}else{
						if (currentAction==2 || currentAction==4 || currentAction==6 || currentAction==8){  //go to door
							cout<<"in policy go door"<<endl;
							newModelName=topomap.goCrossDoorFunc(currentModelName,goMsgPolicy.data);
							if (newModelName=="collision"){
								return -1;
							}	
						}else{
							cout<<"ERROR: invalid action."<<endl;
							return -1;
						}
					}
					
					//localTarget found
					if (newModelName.find(localTarget) !=string::npos ){
						policyReady=0;	
					}
		
					ros::spinOnce();
					loop_rate.sleep();
					
					cout<<"in policy stoping robot"<<endl;
					
			}

			// //Keep exploring choosing a random edge
			
			if ((topomap.fatherChild[1]!=globalTarget1 || topomap.fatherChild[1]!=globalTarget2)  && policyReady==0){
					tempNode=topomap.visitedLocations[topomap.findFatherIndex(topomap.fatherChild[1])];
					cout<<endl<<"Keep exploring..."<<endl;
					if (topomap.getDirection(direction)==-1){
						cout<<"ERROR: direction keep exploring"<<endl;
						return -1;
					}
					
					int resultNextEdge=topomap.goNextEdge(tempNode,direction[1]);
					switch (resultNextEdge){
						case 1:
							cout<<"next node reached"<<endl;
							break;
						case -1:
							cout<<"ERROR: goNextEdge "<<endl;
							return -1;
							break;
						case 2:
							cout<<"AWESOME!! goal found a! "<<endl;
							return 2;
							break;
						case -2:
							cout<<"collision: goNextEdge"<<endl;	//collision exit
							return -2;
							break;
					}
					
					topomap.stopRobot();
					ros::spinOnce();
					loop_rate.sleep();
			
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

