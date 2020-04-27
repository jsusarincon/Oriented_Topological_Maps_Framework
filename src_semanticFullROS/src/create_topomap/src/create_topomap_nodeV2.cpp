#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "create_topomap_node.hh"
#include<limits>
#include <math.h>
#include <fstream>
#include <random>
#include "geometry_msgs/Twist.h"


#include <pwd.h>

#define _USE_MATH_DEFINES

DirectionalGraph::DirectionalGraph(){
    //countFathers=-1;
    lastFatherIndex=-1;
    lastFathersDirection=-1;
    LEFTNUMBER=1;
    RIGHTNUMBER=2;
    getDirectionManually=0; //0 if we want automatic direction obtained from odom, 1 for manual human direction

    sub_objects = n_.subscribe("/objectsDetected", 1, &DirectionalGraph::readObjectsDetected, this);
    sub_direction=n_.subscribe("/direction_robot", 1, &DirectionalGraph::readDirection, this);
    pub_cmd_vel = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    

}

void DirectionalGraph::printGraph(){
	vector<NodeSemantic>  myTempvector;
	NodeSemantic tempNode;
	map <string, int> :: iterator itr;

	cout << "\n \nprint left graph"<<endl;

	for (vector< vector<NodeSemantic> >::iterator it=adjacencyList2Left.begin(); it !=adjacencyList2Left.end(); ++it){
		myTempvector=*it;
		for (vector<NodeSemantic>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			tempNode=*it2;
			cout << tempNode.name << " #edges: "<< tempNode.edges.size()<<" ";
			for (itr = tempNode.visitedEdges.begin(); itr != tempNode.visitedEdges.end(); ++itr){
				cout<<""<< itr->first <<"-"<< itr->second<<endl;
			}
		}
		cout << endl <<endl;
	}
	cout << "print right graph"<<endl;
	for (vector< vector<NodeSemantic> >::iterator it=adjacencyList2Right.begin(); it !=adjacencyList2Right.end(); ++it){
		myTempvector=*it;
		for (vector<NodeSemantic>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			tempNode=*it2;
			cout << tempNode.name << " "<< " #edges: "<< tempNode.edges.size()<<" ";
			for (itr = tempNode.visitedEdges.begin(); itr != tempNode.visitedEdges.end(); ++itr){
				cout <<" "<< itr->first <<"-"<< itr->second<<endl;
			}
		}
		cout << endl<<endl;
	}
}

void DirectionalGraph::saveListsToFile(){
	ofstream outfile1,outfile2,outfileMatrixLeft,outfileMatrixRight, outfileListNodes ;
	const char *homedir;
	vector<NodeSemantic>  myTempvector;

	int matrixLeft[visitedLocations.size()][visitedLocations.size()]={};
	int matrixRight[visitedLocations.size()][visitedLocations.size()]={};
	map<string,int> dictionaryNodes;


	string s1("/media/sf_sharedFolderLinuxSemantic/");	
	string leftFileNameFinal=s1+"leftList.txt";
	string rightFileNameFinal=s1+"rightList.txt";
	string matrixLeftFileNameFinal=s1+"adjMatrixLeft.txt";
	string matrixRightFileNameFinal=s1+"adjMatrixRight.txt";
	string listNodesLeftFileNameFinal=s1+"listNodes.txt";

	outfile1.open(leftFileNameFinal.c_str());
	outfile2.open(rightFileNameFinal.c_str());
	outfileMatrixLeft.open(matrixLeftFileNameFinal.c_str());
	outfileMatrixRight.open(matrixRightFileNameFinal.c_str());
	outfileListNodes.open(listNodesLeftFileNameFinal.c_str());

		
	//creating dictionary (map) of nodes with indexes
	for(int i=0; i<visitedLocations.size(); i++){
		
		dictionaryNodes[visitedLocations[i]]=i;
		outfileListNodes << visitedLocations[i] <<endl;
		//cout<<"visitedLocations[i] "<<visitedLocations[i] <<endl;
	}


	cout << "saving left graph"<<endl;
	int nodeIndexTemp=-1;
	int nodeIndexTemp2=-1;
	for(int i=0; i<adjacencyList2Left.size(); i++){
		myTempvector=adjacencyList2Left[i];
		nodeIndexTemp=dictionaryNodes[myTempvector[0].name];
		for(int j=1; j<myTempvector.size(); j++){  				//we analize the nodes after the first one j=1 and not 0 because of that
			outfile1 << myTempvector[j].name << " ";					//adjacency list
			nodeIndexTemp2=dictionaryNodes[myTempvector[j].name];	//adjancency matrix
			//cout<< "size: "<<visitedLocations.size()<<" matrix left dic "<<nodeIndexTemp<<" "<<nodeIndexTemp2<<endl;
			matrixLeft[nodeIndexTemp][nodeIndexTemp2]=1;		//adjancency matrix
		}
		outfile1 << endl;	//adjacency list
	}
	cout << "saving right graph"<<endl;
	for(int i=0; i<adjacencyList2Right.size(); i++){
		myTempvector=adjacencyList2Right[i];
		nodeIndexTemp=dictionaryNodes[myTempvector[0].name];
		for(int j=1; j<myTempvector.size(); j++){			//we analize the nodes after the first one j=1 and not 0 because of that
			outfile1 << myTempvector[j].name << " "; 			//adjacency list
			nodeIndexTemp2=dictionaryNodes[myTempvector[j].name]; //adjancency matrix
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

int  DirectionalGraph::addEdge(string currentModelName, int *direction, int lastEdgeAdded, int lastNodeAddedIndex){
	
	int lOrR;
	int tempDirection[2];
	int newNodeAddedIndex;

	vector<NodeSemantic> tempVector;

	tempDirection[0]=direction[0];
	tempDirection[1]=direction[1];
	lOrR=direction[0];
	

	NodeSemantic newNode,oldNode;

	if (lastEdgeAdded==RIGHTNUMBER && lOrR==RIGHTNUMBER){

		//Update Left
		tempDirection[0]=LEFTNUMBER;
		newNodeAddedIndex=addNode(currentModelName,tempDirection);
		scanForEdges(direction[1],true,tempDirection[0], currentModelName,newNodeAddedIndex);


		//update visitedEdges
		updateVisitedEdges(lOrR,lastNodeAddedIndex,direction,newNodeAddedIndex);

		//Update Left continue
		oldNode=adjacencyList2Right[lastNodeAddedIndex][0];
		adjacencyList2Left[newNodeAddedIndex].push_back(oldNode);
		
		//Update Right
		newNode=adjacencyList2Left[newNodeAddedIndex][0];
		adjacencyList2Right[lastNodeAddedIndex].push_back(newNode);

		cout<<"yyyyyy"<<endl;
		//lastNodeAddedIndex=addNode(modelName,direction);

		tempVector.push_back(newNode);
		adjacencyList2Right.push_back(tempVector);
		lastNodeAddedIndex=adjacencyList2Right.size()-1;
		
	}else{
		if (lastEdgeAdded==LEFTNUMBER && lOrR==LEFTNUMBER){
			//Update Right
			tempDirection[0]=RIGHTNUMBER;
			newNodeAddedIndex=addNode(currentModelName,tempDirection);
			scanForEdges(direction[1],true,tempDirection[0], currentModelName,newNodeAddedIndex);

			oldNode=adjacencyList2Left[lastNodeAddedIndex][0];
			adjacencyList2Right[newNodeAddedIndex].push_back(oldNode);

			
			//Update Left
			newNode=adjacencyList2Right[newNodeAddedIndex][0];
			adjacencyList2Left[lastNodeAddedIndex].push_back(newNode);
			//lastNodeAddedIndex=addNode(modelName,direction);
			tempVector.push_back(newNode);
			adjacencyList2Left.push_back(tempVector);
			lastNodeAddedIndex=adjacencyList2Left.size()-1;
		}else{
			if (lastEdgeAdded==RIGHTNUMBER && lOrR==LEFTNUMBER){
					cout<<"cttt: "<<lastNodeAddedIndex<<endl;
					//Update Right 
					tempDirection[0]=RIGHTNUMBER;
					newNodeAddedIndex=addNode(currentModelName,tempDirection);
					scanForEdges(direction[1],true,tempDirection[0], currentModelName,newNodeAddedIndex);
					oldNode=adjacencyList2Right[lastNodeAddedIndex][0];
					adjacencyList2Right[newNodeAddedIndex].push_back(oldNode);

					//Update Left
					
					tempVector.push_back(oldNode);
					adjacencyList2Left.push_back(tempVector);
					lastNodeAddedIndex=adjacencyList2Left.size()-1;
					newNode=adjacencyList2Right[newNodeAddedIndex][0];
					adjacencyList2Left[lastNodeAddedIndex].push_back(newNode);
					//lastNodeAddedIndex=addNode(modelName,direction);
					tempVector.push_back(newNode);
					adjacencyList2Left.push_back(tempVector);

			}else{
				if (lastEdgeAdded==LEFTNUMBER && lOrR==RIGHTNUMBER){
					cout<<"ca: "<<lastNodeAddedIndex<<endl;

					//Update Left
					tempDirection[0]=LEFTNUMBER;
					newNodeAddedIndex=addNode(currentModelName,tempDirection);
					scanForEdges(direction[1],true,tempDirection[0], currentModelName,newNodeAddedIndex);
					oldNode=adjacencyList2Left[lastNodeAddedIndex][0];
					adjacencyList2Left[newNodeAddedIndex].push_back(oldNode);

					//Update Right 
					
					tempVector.push_back(oldNode);
					adjacencyList2Right.push_back(tempVector);
					lastNodeAddedIndex=adjacencyList2Right.size()-1;
					newNode=adjacencyList2Left[newNodeAddedIndex][0];
					adjacencyList2Right[lastNodeAddedIndex].push_back(newNode);
					//lastNodeAddedIndex=addNode(modelName,direction);
					tempVector.push_back(newNode);
					adjacencyList2Right.push_back(tempVector);
				}
			}
		}
	}
	//updateFather(modelName,direction);
	return lastNodeAddedIndex;
}


int DirectionalGraph::findFatherIndex(string currentModelName, int lOrR){
	vector<NodeSemantic>  myTempvector;
	
	if (lOrR==RIGHTNUMBER){
    		//cout << "searching right graph"<<endl;
		for (int i=0;i<adjacencyList2Right.size();i++){
			myTempvector=adjacencyList2Right[i];
			if (myTempvector[0].name==currentModelName){
				return i;
			}
		}
	}else{
			//cout << "searching left graph"<<endl;
		for (int i=0;i<adjacencyList2Left.size();i++){
			myTempvector=adjacencyList2Left[i];
			if (myTempvector[0].name==currentModelName){
				return i;
			}
		}
	}
	return -1; //if index not found
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


		if (mydirection_robot >=-M_PI/8 && mydirection_robot<=M_PI/8){
			direction[0]=2; //indicates that the new place is on the right of the robot
			direction[1]=1; //indicates that the robot is facing east
			
		}else{
			if (mydirection_robot >3*M_PI/8 && mydirection_robot<=5*M_PI/8){
				direction[0]=2; //indicates that the new place is on the right of the robot
				direction[1]=2; //indicates that the robot is facing north
				
			}else{
				if (mydirection_robot >7*M_PI/8 && mydirection_robot <=M_PI || mydirection_robot<=-7*M_PI/8 && mydirection_robot>=-M_PI){
					direction[0]=1; //indicates that the new place is on the left of the robot
					direction[1]=3; //indicates that the robot is facing west
					
				}else{
					if (mydirection_robot >-5*M_PI/8 && mydirection_robot<-3*M_PI/8){
						direction[0]=1; //indicates that the new place is on the left of the robot
						direction[1]=4; //indicates that the robot is facing south
						
					}
				}
			}
		}
	}
	//cout<<"direction: "<<direction[0] << " compass: " <<direction[1]<<endl;
	
}

int DirectionalGraph::addNode(string currentModelName, int *direction){
	int lastNodeAddedIndex=-1;
	int lOrR;
	vector<NodeSemantic> tempVector;
	NodeSemantic newNode;
	newNode.name=currentModelName;
	newNode.direction=direction[1];
	newNode.finishedScaningEdgesFlag=false;
	lOrR=direction[0];
	tempVector.push_back(newNode);
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
			
			lastFatherName=currentModelName;
			lOrR=direction[0];
			lastFathersDirection=lOrR;
			if (lOrR==LEFTNUMBER){
				lastFatherIndex=findFatherIndex( currentModelName,  lOrR);
				//cout <<"zzzz: "<<adjacencyList2Left.size()<<" lastFatherIndex: "<<lastFatherIndex<<endl;
			}else{
				lastFatherIndex=findFatherIndex( currentModelName,  lOrR);;
				//cout <<"ddddd: "<<adjacencyList2Right.size()<<" lastFatherIndex: "<<lastFatherIndex<<endl;
			}
			//cout <<"lastFatherName1: "<<lastFatherName<<" lastFatherIndex: "<<lastFatherIndex<<endl;
			//return currentModelName;
	}
}
void DirectionalGraph::scanForEdges(int initialOri,bool equalDirec,int lOrR,string myfirstModelName,int lastNodeAddedIndex){
	float currentOri;
	int direction[2]={};
	ros::Rate loop_rate(2);
	
	//move until the logical camera stop seeing the object to start rotating and scaning for intersections
	//this is to try to center the robot on top of the tag before scaning
	// for (int i=1; i<3;i++){
	// 	moveRobot(0.7,0);
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }
	//move until it doesnt see corridor or se1 to execute maneuvers
	string outNode=myfirstModelName;

	while(outNode.find("corridor") !=string::npos || outNode.find("se1") !=string::npos){
		//cout<<"out of the model: "<< currentModelName<<endl;
		moveRobot(0.5,0);
		outNode=modelName;
		ros::spinOnce();
		loop_rate.sleep();
	}
	stopRobot();



	//rotate until the first orientation changes but still check if there is intersections
	int sizeRow;
	while(equalDirec==true){
		getDirection(direction);
		currentOri=direction[1];
		moveRobot(0,-0.5);
		//cout<<"scaning for edges 1: "<<modelName<<endl;
		if (modelName.find("intersection") !=string::npos){
			if (lOrR==LEFTNUMBER){
				adjacencyList2Left[lastNodeAddedIndex][0].edges[modelName]=direction[1];
				adjacencyList2Left[lastNodeAddedIndex][0].visitedEdges[modelName]=0;
			}else{
				adjacencyList2Right[lastNodeAddedIndex][0].edges[modelName]=direction[1];
				adjacencyList2Right[lastNodeAddedIndex][0].visitedEdges[modelName]=0;	
			}
		}

		if (initialOri-currentOri!=0)
			break;
		ros::spinOnce();
		loop_rate.sleep();
	}
	//rotate the rest of the 360 degrees to cover the whole area for possible intersections
	equalDirec=false;
	while(equalDirec==false){
		getDirection(direction);
		currentOri=direction[1];
		moveRobot(0,-0.5);
		//cout<<"scaning for edges 2: "<<modelName<<endl;
		if (modelName.find("intersection") !=string::npos){
			if (lOrR==LEFTNUMBER){
				adjacencyList2Left[lastNodeAddedIndex][0].edges[modelName]=direction[1];
				adjacencyList2Left[lastNodeAddedIndex][0].visitedEdges[modelName]=0;
				adjacencyList2Left[lastNodeAddedIndex][0].finishedScaningEdgesFlag=true;
			}else{
				adjacencyList2Right[lastNodeAddedIndex][0].edges[modelName]=direction[1];
				adjacencyList2Right[lastNodeAddedIndex][0].visitedEdges[modelName]=0;
				adjacencyList2Right[lastNodeAddedIndex][0].finishedScaningEdgesFlag=true;
			}
		}

		if (initialOri-currentOri==0)
			break;
		ros::spinOnce();
		loop_rate.sleep();

	}
	stopRobot();

}
/*
void DirectionalGraph::createGraph(string modelName, int *direction){

	
			}else{	//if the location found was already added but the edge with the last location hasnt been created (closing the building to connect nodes first with last one)
				if (lastFatherName!=modelName){	//if the new place is differente from the last place visited then find edge, otherwise the robot is observing the same node and there is not edge with itself
					if (findEdge(modelName,lastFatherName)==false){ //edge no found then add it
						//getDirection(direction);
						lOrR=direction[0];
						lastFatherIndex=addEdge(modelName,lOrR);
						printGraph();
						
						cout <<"lastFatherName2: "<<lastFatherName<<endl;
					}else{ //update father when detecting the place
						cout <<"update father: "<<lastFatherName<<endl;
						//getDirection(direction);
						lOrR=direction[0];

						lastFatherIndex=findFatherIndex(modelName,lOrR);
						lastFatherName=modelName;
						lastFathersDirection=lOrR;
					}
					printGraph();

*/
void DirectionalGraph::readObjectsDetected(const create_topomap::logicalImage::ConstPtr& msg){
  //cout <<"I heard: "<<msg->modelName<<endl;
  	
	int compass;
	int direction[2]={};
	getDirection(direction);
	compass=direction[1];
	stringstream ss;
	ss << compass;
	
	modelName=msg->modelName; //model name 
	if ((modelName.find("corridor") !=string::npos || modelName.find("se1") !=string::npos || modelName.find("intersection") !=string::npos) && compass!=0){ //find the word corridor or se1 in the modelName and ignore the other objects
		modelName=msg->modelName+ss.str(); //model name plus compass reference
	}else{
		modelName="Other object";
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
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.angular.z = 0;    
	pub_cmd_vel.publish(msg);
}

void DirectionalGraph::faceNewDirection(int newDirection){

	cout<<"faceFront"<<endl;
	ros::Rate loop_rate(10);
	moveRobot(0,0.5);
	while (1){

		if (newDirection==1 && (mydirection_robot >=-M_PI/8 && mydirection_robot<=M_PI/8)){ //east
			break;
		}else{
			if (newDirection==2 && (mydirection_robot >3*M_PI/8 && mydirection_robot<=5*M_PI/8)){ //north
				break;
				
			}else{
				//west
				if (newDirection==3 && (mydirection_robot >7*M_PI/8 && mydirection_robot <=M_PI || mydirection_robot<=-7*M_PI/8 && mydirection_robot>=-M_PI)){
					break;
					
				}else{
					if (newDirection==4 && (mydirection_robot >-5*M_PI/8 && mydirection_robot<-3*M_PI/8)){ //south
						break;
					}
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
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
void DirectionalGraph::updateVisitedEdges(int lOrR,int lastNodeAddedIndex,int *direction,int newNodeAddedIndex){
	
	NodeSemantic newNode,oldNode;
	map<string, int>::iterator it;
	map <string, int> :: iterator itr;
	int myOpositeDirection;

	//update visitedEdges
	if (lOrR==RIGHTNUMBER){
		cout<<endl<<"UPDATING VISITED"<<endl<<endl;
		//updating previous node
		oldNode=adjacencyList2Right[lastNodeAddedIndex][0];
		cout<<"oldNode "<<oldNode.name<<endl;
		for (itr = oldNode.edges.begin(); itr != oldNode.edges.end(); ++itr){
			if (itr->second==direction[1]){
				cout<<"aca "<<endl;
				it = adjacencyList2Right[lastNodeAddedIndex][0].visitedEdges.find(itr->first); 

				//if (it != oldNode.edges.end())
				    it->second = 1;
			}
		}

		
		//updating new node
		newNode=adjacencyList2Left[newNodeAddedIndex][0];
		cout<<"newNode "<<newNode.name<<endl;
		printGraph();
		//if the robot is facing east then it means the edge west is visited because it is comming from the left
		myOpositeDirection=opositeDirection(direction[1]);
		for (itr = newNode.edges.begin(); itr != newNode.edges.end(); ++itr){
			if (itr->second==myOpositeDirection){
				cout<<"alli "<<endl;
				it = adjacencyList2Left[newNodeAddedIndex][0].visitedEdges.find(itr->first); 
				//if (it != newNode.edges.end())
				    it->second = 1;
				cout<<"first: "<<itr->first<<" second: "<<it->second<<endl;

			}
		}
		printGraph();
	}else{
		//updating previous node
		oldNode=adjacencyList2Left[lastNodeAddedIndex][0];
		
		for (itr = oldNode.edges.begin(); itr != oldNode.edges.end(); ++itr){
			if (itr->second==direction[1]){
				it = adjacencyList2Left[newNodeAddedIndex][0].visitedEdges.find(itr->first); 
				//if (it != oldNode.edges.end())
				    it->second = 1;
			}
		}

		//updating new node
		newNode=adjacencyList2Right[newNodeAddedIndex][0];
		//if the robot is facing east then it means the edge west is visited because it is comming from the left
		myOpositeDirection=opositeDirection(direction[1]);
				
		for (itr = newNode.edges.begin(); itr != newNode.edges.end(); ++itr){
			if (itr->second==myOpositeDirection){
				it = adjacencyList2Right[lastNodeAddedIndex][0].visitedEdges.find(itr->first); 
				//if (it != newNode.edges.end())
				    it->second = 1;
			}
		}
	}

}

bool isFileExist(const char *fileName){
    std::ifstream infile(fileName);
    return infile.good();
}

int main(int argc, char **argv){
	bool startExploringFlag=true;
	int lOrR=0; //left or right
	int tempFatherIndex;
	int direction[2]={}; //[0]=left(1) of right(2), [1]=East(1), North(2), West(3), South(4)
	int directionTemp[2]={};
	int lastNodeAddedIndex, lastNodeAddedIndexPast;
	string currentModelName,lastNodeSeen;
	int lastEdgeAdded=-1;
	int policyReady=0;
	NodeSemantic tempNode;

	//ROS
	ros::init(argc, argv, "create_topomap_node");
	ros::NodeHandle n2;
	ros::Publisher goCorridor = n2.advertise<std_msgs::Int16>("go_corridor", 1);
	ros::Publisher goDoor = n2.advertise<std_msgs::Int16>("go_cross_door", 1);

	ros::Rate loop_rate(10);


	//Semantic map
	DirectionalGraph topomap;

	topomap.modelName="Other object";


	while (ros::ok()){

		//Start exploration
		while(startExploringFlag==true){
			topomap.moveRobot(0,-0.5);
			 ros::spinOnce();
			 loop_rate.sleep();
			cout<<"Searching for first node"<<endl;
			topomap.getDirection(direction);
			topomap.updateFather(topomap.modelName,direction);
			currentModelName=topomap.modelName;
			if (currentModelName.find("corridor") !=string::npos || currentModelName.find("se1") !=string::npos || currentModelName.find("intersection") !=string::npos){ //find the word corridor or se1 in the modelName and ignore the other objects
				
				if (topomap.adjacencyList2Left.size()==0 && topomap.adjacencyList2Right.size()==0 && currentModelName.find("intersection") ==string::npos){ //0 nodes in the graph
					startExploringFlag=false;
					topomap.stopRobot();
					topomap.visitedLocations.push_back(currentModelName);
					lastNodeAddedIndex=topomap.addNode(currentModelName,direction);
				if (topomap.adjacencyList2Left.size()!=0 || topomap.adjacencyList2Right.size()!=0){}
				
					//move until it sees an intersection 
					while(currentModelName.find("intersection") ==string::npos){
						//cout<<"out of the model: "<< currentModelName<<endl;
						topomap.moveRobot(0.5,0);
						currentModelName=topomap.modelName;
						ros::spinOnce();
						loop_rate.sleep();
					}
					topomap.stopRobot();
					topomap.scanForEdges(direction[1],true,direction[0], currentModelName,lastNodeAddedIndex);
					lastEdgeAdded=direction[0];
					cout<<"print1"<<endl;
					topomap.printGraph();
				}
			}
		}

		

		//cout<<"eddd"<<topomap.modelName<<endl;
		//cout <<"qqqqq: "<<direction[0]<<direction[1]<<endl;
		//topomap.updateFather(topomap.modelName,direction);
		currentModelName=topomap.modelName;
		if (currentModelName.find("corridor") !=string::npos || currentModelName.find("se1") !=string::npos ){ //find the word corridor or se1 in the modelName and ignore the other objects
			if (find(topomap.visitedLocations.begin(), topomap.visitedLocations.end(), currentModelName) == topomap.visitedLocations.end()) { //if new model found
					topomap.visitedLocations.push_back(currentModelName);
					topomap.getDirection(direction);
					//tempFatherIndex=topomap.findFatherIndex(currentModelName,lOrR);
					//if (tempFatherIndex==-1){ //modelName no found		
					cout<<"addedNode "<<lastNodeAddedIndex<<" "<<lastEdgeAdded<<endl;
					lastNodeAddedIndex=topomap.addEdge(currentModelName,direction, lastEdgeAdded, lastNodeAddedIndex);
					lastEdgeAdded=direction[0];
					//}else{ //add new node to the existing node
						//topomap.addEdge(currentModelName,direction, lastEdgeAdded);
						//topomap.addEdge(currentModelName,tempFatherIndex,direction);
					//	lastEdgeAdded=direction[0];
					//	cout<<"aca"<<endl;
					//}
					topomap.printGraph();
					
			}else{ //node already added
				lastNodeAddedIndex=topomap.findFatherIndex(currentModelName,direction[0]);
				cout<<"addedNode2 "<<lastNodeAddedIndex<<currentModelName<<endl;
			}
		}


		//Creating inital graph with at least 3 nodes

		if (topomap.visitedLocations.size()<3){
			directionTemp[0]=direction[0];
			directionTemp[1]=direction[1];
			topomap.getDirection(direction);
			
			if (lastEdgeAdded==topomap.RIGHTNUMBER){
				tempNode=topomap.adjacencyList2Right[lastNodeAddedIndex][0];
			}else{
				tempNode=topomap.adjacencyList2Left[lastNodeAddedIndex][0];
			}

			int numEdges=tempNode.edges.size();
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
				cout<<"newDirection "<<newDirection<<endl;
				if (newDirection!=tempNode.direction){ //if direction of the picked now is different from the current direction the robot needs to rotate
					topomap.faceNewDirection(newDirection);

				}
				std_msgs::Int16 goMsg;
				goMsg.data=1;
				lastNodeSeen=topomap.visitedLocations[topomap.visitedLocations.size()-1];
				cout<<"lastNodeSeen: "<<lastNodeSeen<<endl;
				if (lastNodeSeen.find("corridor") !=string::npos && newDirection==directionTemp[1]) { //if robot is pointing
					while(topomap.modelName.find("corridor") ==string::npos ){
							//cout<<"modelName 1: "<<topomap.modelName<<endl;
							goCorridor.publish(goMsg);
							ros::spinOnce();
						loop_rate.sleep();
					}
				}else{
					goMsg.data=newDirection;
					if (lastNodeSeen.find("se1") !=string::npos && newDirection!=directionTemp[1]){
						while(topomap.modelName.find("corridor") ==string::npos){
								cout<<"modelName 2: "<<topomap.modelName<<endl;
								goDoor.publish(goMsg);
								ros::spinOnce();
								loop_rate.sleep();
						}
					}
				}
				goMsg.data=0;
				goCorridor.publish(goMsg);
				goDoor.publish(goMsg);
			}
		}else{ //There is more than 3 nodes so we can use the CMDP to explore
			if (policyReady==0){
				//Write files to be used by matlab
				ofstream startFile,targetFile;
				string s1("/media/sf_sharedFolderLinuxSemantic/");
				string startNameFile=s1+"start.txt";
				string targetNameFile=s1+"target.txt";
				startFile.open(startNameFile.c_str());
				targetFile.open(targetNameFile.c_str());
				if (lastEdgeAdded==1){ //last node added to the left
					tempNode=topomap.adjacencyList2Left[lastNodeAddedIndex][0];
				}else{
					if (lastEdgeAdded==2){ //last node added to the right
						tempNode=topomap.adjacencyList2Right[lastNodeAddedIndex][0];
					}
				}
				startFile << tempNode.name<<endl;
				targetFile<<"corridor_1a1"<<endl;
				topomap.saveListsToFile();
				startFile.close();
				targetFile.close();
				

				//wait until the policy is computed and we can read it. if the file endWrinting exist then it means matlab finished writing the policy into files
				while ( isFileExist("/media/sf_sharedFolderLinuxSemantic/endWriting.txt") ){
					policyReady=1;
					string line;
					vector<string> statesCMDP;
					vector<int> actionsCMDP;
					ifstream statesfile ("/media/sf_sharedFolderLinuxSemantic/statesPolicy.txt");
					ifstream actionsfile ("/media/sf_sharedFolderLinuxSemantic/actionsPolicy.txt");
					if (statesfile.is_open()){
						while ( getline (statesfile,line) ){
							cout << line << '\n';
							statesCMDP.push_back(line);
						}
						statesfile.close();
					}else{
						cout << "Unable to open file states"; 
					}

					if (actionsfile.is_open()){
						while ( getline (actionsfile,line) ){
							cout << line << '\n';
							actionsCMDP.push_back(stoi(line));
						}
						actionsfile.close();
					}else{
						cout << "Unable to open file actions"; 
					}

					//delete endWriting File to know it was read correctly
					if( remove( "/media/sf_sharedFolderLinuxSemantic/endWriting.txt" ) != 0 )
					  perror( "Error deleting file" );
					else
					  puts( "File successfully deleted" );

					//move robot based on policy
					int indexCurrentState, currentAction,directionCurrentState;
					
					currentModelName=tempNode.name;
					for (int i=0; i<statesCMDP.size();i++){
						if (currentModelName.find(statesCMDP[i]) !=string::npos){
							indexCurrentState=i;
							break;
						}
					}
					currentAction=actionsCMDP[indexCurrentState];
					cout<<"currentAction "<<currentAction<<" "<<currentModelName<<endl;
					//speed  maneuver
					//   1       1       slow stayCenter
					//   1       2       slow crossdoor
					//   3       1       fast stayCenter
					//   3       2       slow crossdoor
					//numberActionsLeft=[1 2 3 4];
					//numberActionsRight=[5 6 7 8];

					directionCurrentState=tempNode.direction;
					
					if (direction[1]!=directionCurrentState){ //if direction of the picked now is different from the current direction the robot needs to rotate
							topomap.faceNewDirection(directionCurrentState);
							cout<<"directionCurrentStateAA "<<directionCurrentState<<endl;
					}
					if (direction[0]==topomap.RIGHTNUMBER && (currentAction==1 || currentAction==2 || currentAction==3 || currentAction==4)){
						topomap.faceNewDirection(topomap.opositeDirection(directionCurrentState));
						cout<<"directionCurrentStateBB "<<directionCurrentState<<endl;
					} 
					if (direction[0]==topomap.LEFTNUMBER && (currentAction==5 || currentAction==6 || currentAction==7 || currentAction==8)){
						topomap.faceNewDirection(topomap.opositeDirection(directionCurrentState));
						cout<<"directionCurrentStateCC "<<directionCurrentState<<endl;
					} 

					std_msgs::Int16 goMsgPolicy;
					
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

					cout<<"goMsgPolicy.data  "<<goMsgPolicy.data<<endl;

					if (currentAction==1 || currentAction==3 || currentAction==5 || currentAction==7){
						while(topomap.modelName.find("corridor") ==string::npos ){
								//cout<<"modelName 1: "<<topomap.modelName<<endl;
								goCorridor.publish(goMsgPolicy);
								ros::spinOnce();
							loop_rate.sleep();
						}
					}else{
						if (currentAction==2 || currentAction==4 || currentAction==6 || currentAction==8){
							if (currentModelName.find("se1") !=string::npos ){
								while(topomap.modelName.find("corridor") ==string::npos){
										//cout<<"modelName 2: "<<topomap.modelName<<endl;
										goDoor.publish(goMsgPolicy);
										ros::spinOnce();
										loop_rate.sleep();
								}
							}
						}
					}

				}	//finishing moving using CMDP policy


			}
		}
		

		topomap.modelName="No Objects";
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

