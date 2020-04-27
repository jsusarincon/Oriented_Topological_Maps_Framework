//#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "create_topomap/logicalImage.h"
#include "create_topomap_node.hh"
#include<limits>
#include <math.h>
#include <fstream>


#include <pwd.h>

#define _USE_MATH_DEFINES

DirectionalGraph::DirectionalGraph()
{
    //countFathers=-1;
    lastFatherIndex=-1;
    lastFathersDirection=-1;
    LEFTNUMBER=1;
    RIGHTNUMBER=2;
    getDirectionManually=0; //0 if we want automatic direction obtained from odom, 1 for manual human direction

    sub_objects = n_.subscribe("/objectsDetected", 1, &DirectionalGraph::readObjectsDetected, this);
    sub_direction=n_.subscribe("/direction_robot", 1, &DirectionalGraph::readDirection, this);

    //this->V = V;
    //adj = new list<int>[V];

}

void DirectionalGraph::printGraph(){
	vector<string>  myTempvector;

	cout << "\n \nprint left graph"<<endl;

	for (vector< vector<string> >::iterator it=adjacencyList2Left.begin(); it !=adjacencyList2Left.end(); ++it){
		myTempvector=*it;
		for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			cout << *it2 << " ";
		}
		cout << endl;
	}
	cout << "print right graph"<<endl;
	for (vector< vector<string> >::iterator it=adjacencyList2Right.begin(); it !=adjacencyList2Right.end(); ++it){
		myTempvector=*it;
		for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			cout << *it2 << " ";
		}
		cout << endl;
	}
}

void DirectionalGraph::saveListsToFile(){
	ofstream outfile1,outfile2,outfileMatrixLeft,outfileMatrixRight, outfileListNodes ;
	const char *homedir;
	vector<string>  myTempvector;

	int matrixLeft[visitedLocations.size()][visitedLocations.size()]={};
	int matrixRight[visitedLocations.size()][visitedLocations.size()]={};
	map<string,int> dictionaryNodes;

/*  //get the home path if we want to save the files to the home folder
	homedir=getenv("HOME");
	if ( homedir == NULL ) {
    	homedir = getpwuid(getuid())->pw_dir;
	}	
	
    string s1(homedir);
*/
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
	}

	/*for (int i=0; i<visitedLocations.size();i++){
		for(int j=0; j<visitedLocations.size(); j++){
			cout<< "matrix left initial: "<<i<<" "<<j<<": "<<matrixLeft[i][j]<<endl;
		}
	}
	*/
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


	/*for (vector< vector<string> >::iterator it=adjacencyList2Left.begin(); it !=adjacencyList2Left.end(); ++it){
		myTempvector=*it;
		for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			//cout << *it2 << " ";
			outfile1 << *it2 << " ";
		}
		//cout << endl;
		outfile1 << endl;
	}
	cout << "saving right graph"<<endl;
	for (vector< vector<string> >::iterator it=adjacencyList2Right.begin(); it !=adjacencyList2Right.end(); ++it){
		myTempvector=*it;
		for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
			//cout << *it2 << " ";
			outfile2 << *it2 << " ";
		}
		//cout << endl;
		outfile2 << endl;
	}

	*/

	outfile1.close();
	outfile2.close();

	outfileMatrixLeft.close();
	outfileMatrixRight.close();

	outfileListNodes.close();
	

	cout <<"Files Printed"<<endl;
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

		*Each direction is composed by 90 degrees (pi/2), 
		 0 is horizonal x axis that going CCW goes up to pi, 
		 then from horizonal negative x -pi goes to zero again 
					
			1) East goes from -pi/4 to pi/4
			2) North goes from pi/4 to pi*3/4
			3) West goes from pi*3/4 to -pi*3/4
			4) South goes from -3/4*pi/4 to -pi/4
		*/
	//cout<<"mydirection_robot: "<<mydirection_robot<endl;
		if (mydirection_robot >=-M_PI/4 && mydirection_robot<=M_PI/4){
			direction[0]=1; //indicates that the new place is on the left of the robot
			direction[1]=1; //indicates that the robot is facing east

		}else{
			if (mydirection_robot >M_PI/4 && mydirection_robot<=M_PI*3/4){
				direction[0]=1; //indicates that the new place is on the left of the robot
				direction[1]=2; //indicates that the robot is facing north
			}else{
				if (mydirection_robot >M_PI*3/4 && mydirection_robot<=-M_PI*3/4){
					direction[0]=2; //indicates that the new place is on the right of the robot
					direction[1]=3; //indicates that the robot is facing west
				}else{
					if (mydirection_robot >-M_PI*3/4 && mydirection_robot<-M_PI/4)
					direction[0]=2; //indicates that the new place is on the right of the robot
					direction[1]=4; //indicates that the robot is facing south
				}
			}
		}
	}
	//cout<<"direction: "<<direction[0] << " compass: " <<direction[1]<<endl;
	
}

int DirectionalGraph::addNode(string modelName, int lOrR){
	int lastNodeAddedIndex=-1;
	vector<string> tempVector;
	tempVector.push_back(modelName);
	if (lOrR==LEFTNUMBER){
		adjacencyList2Left.push_back(tempVector);
		lastNodeAddedIndex=adjacencyList2Left.size()-1;
	}else{
		adjacencyList2Right.push_back(tempVector);
		lastNodeAddedIndex=adjacencyList2Right.size()-1;
	}
	lastFatherName=modelName;
cout <<"lastFatherName1: "<<lastFatherName<<endl;
    //fathers[modelName] = countFathers;  //
    return lastNodeAddedIndex;
}

int DirectionalGraph::addEdge(string modelName, int lOrR)
{
	int lastNodeAddedIndex=-1;

	if (lastFathersDirection==RIGHTNUMBER && lOrR==RIGHTNUMBER){
		adjacencyList2Right[lastFatherIndex].push_back(modelName);
		lastNodeAddedIndex=addNode(modelName,LEFTNUMBER);
		lastFatherName=adjacencyList2Right[lastFatherIndex][0];
		adjacencyList2Left[lastNodeAddedIndex].push_back(lastFatherName);
	}else{
		if (lastFathersDirection==LEFTNUMBER && lOrR==LEFTNUMBER){
			adjacencyList2Left[lastFatherIndex].push_back(modelName);
			lastNodeAddedIndex=addNode(modelName,RIGHTNUMBER);
			lastFatherName=adjacencyList2Left[lastFatherIndex][0];
			adjacencyList2Right[lastNodeAddedIndex].push_back(lastFatherName);
		}else{
			if (lastFathersDirection==RIGHTNUMBER && lOrR==LEFTNUMBER){

				lastFatherName=adjacencyList2Right[lastFatherIndex][0];
				lastNodeAddedIndex=addNode(lastFatherName,LEFTNUMBER);
				adjacencyList2Left[lastNodeAddedIndex].push_back(modelName);
				lastFathersDirection=lOrR;		
				//lastNodeAddedIndex=addNode(lastFatherName,RIGHTNUMBER);
			
			}else{
				if (lastFathersDirection==LEFTNUMBER && lOrR==RIGHTNUMBER){
					lastFatherName=adjacencyList2Left[lastFatherIndex][0];
					lastNodeAddedIndex=addNode(lastFatherName,RIGHTNUMBER);
					adjacencyList2Right[lastNodeAddedIndex].push_back(modelName);	
					lastFathersDirection=lOrR;				
					//addNode(lastFatherName,LEFTNUMBER);
				}
			}
		}
	}
	lastFatherName=modelName;
	return lastNodeAddedIndex;

}

void DirectionalGraph::addEdge(string modelName, int lOrR, int fatherIndex)
{

	int lastNodeAddedIndex=-1;

	if (lastFathersDirection==RIGHTNUMBER && lOrR==RIGHTNUMBER){
		adjacencyList2Right[fatherIndex].push_back(modelName);
		lastNodeAddedIndex=addNode(modelName,LEFTNUMBER);
		lastFatherName=adjacencyList2Right[lastFatherIndex][0];
		adjacencyList2Left[lastNodeAddedIndex].push_back(lastFatherName);
	}else{
		if (lastFathersDirection==LEFTNUMBER && lOrR==LEFTNUMBER){
			adjacencyList2Left[fatherIndex].push_back(modelName);
			lastNodeAddedIndex=addNode(modelName,RIGHTNUMBER);
			lastFatherName=adjacencyList2Left[lastFatherIndex][0];
			adjacencyList2Right[lastNodeAddedIndex].push_back(lastFatherName);
		}else{
			if (lastFathersDirection==RIGHTNUMBER && lOrR==LEFTNUMBER){
				adjacencyList2Right[fatherIndex].push_back(modelName);

				lastFatherName=adjacencyList2Right[lastFatherIndex][0];
				lastNodeAddedIndex=addNode(lastFatherName,LEFTNUMBER);
				adjacencyList2Left[lastNodeAddedIndex].push_back(modelName);
				lastFathersDirection=lOrR;		
				//lastNodeAddedIndex=addNode(lastFatherName,RIGHTNUMBER);
			
			}else{
				if (lastFathersDirection==LEFTNUMBER && lOrR==RIGHTNUMBER){
					adjacencyList2Left[fatherIndex].push_back(modelName);
					lastFatherName=adjacencyList2Left[lastFatherIndex][0];
					lastNodeAddedIndex=addNode(lastFatherName,RIGHTNUMBER);
					adjacencyList2Right[lastNodeAddedIndex].push_back(modelName);	
					lastFathersDirection=lOrR;				
					//addNode(lastFatherName,LEFTNUMBER);
				}
			}
		}
	}
	lastFatherName=modelName;

	if (lOrR==LEFTNUMBER){
		adjacencyList2Left[fatherIndex].push_back(modelName);
	}else{
		adjacencyList2Right[fatherIndex].push_back(modelName);
	}

	lastFatherName=modelName;
}

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

int DirectionalGraph::findFatherIndex(string modelName, int lOrR){
	vector<string>  myTempvector;
	
	if (lOrR==RIGHTNUMBER){
    		//cout << "searching right graph"<<endl;
		for (int i=0;i<adjacencyList2Right.size();i++){
			myTempvector=adjacencyList2Right[i];
			if (myTempvector[0]==modelName){
				return i;
			}
		}
	}else{
			//cout << "searching left graph"<<endl;
		for (int i=0;i<adjacencyList2Left.size();i++){
			myTempvector=adjacencyList2Left[i];
			if (myTempvector[0]==modelName){
				return i;
			}
		}
	}
	return -1; //if index not found
}

void DirectionalGraph::createGraph(string modelName, int *direction){

	int lOrR=0;
	int tempFatherIndex;
	if (modelName.find("corridor") !=string::npos || modelName.find("se1") !=string::npos ){ //find the word corridor or se1 in the modelName and ignore the other objects
		
		if (adjacencyList2Left.size()==0 && adjacencyList2Right.size()==0){ //0 nodes in the graph
			
			getDirection(direction);
			visitedLocations.push_back(modelName);
			cout<<"zzzzzzdirection: "<< direction[0]<< " "<< direction[1]<<endl;
			lOrR=direction[0];
			addNode(modelName,lOrR);

    		lastFathersDirection=lOrR;
    		
    		if (lOrR==LEFTNUMBER){
    			lastFatherIndex=adjacencyList2Left.size()-1;
    		}else{
    			lastFatherIndex=adjacencyList2Right.size()-1;
    		}
    		printGraph();
			
		}else{		
			if (find(visitedLocations.begin(), visitedLocations.end(), modelName) == visitedLocations.end()) { //if new model found
				visitedLocations.push_back(modelName);
				//getDirection(direction);
				lOrR=direction[0];

				tempFatherIndex=findFatherIndex(modelName,lOrR);
				if (tempFatherIndex==-1){					
					addEdge(modelName,lOrR);
					addNode(modelName,lOrR);

					if (lOrR==LEFTNUMBER){
	    				lastFatherIndex=adjacencyList2Left.size()-1;
	    			}else{
	    				lastFatherIndex=adjacencyList2Right.size()-1;
	    			}
	    		}else{ //add new node to the existend node
					addEdge(modelName,lOrR,tempFatherIndex);
	    		}
	    		printGraph();
	    		saveListsToFile();
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
					
				}

			}
			
		}

		
	}

/*
    bool foundIndex=0;
    
        if (modelName.find("corridor") !=string::npos ){ //find the word corridor in the modelName
            
            if (adjacencyList.size()>0){
                for (vector< vector<string> >::iterator it=adjacencyList.begin(); it !=adjacencyList.end(); ++it){
                    vector<string>  myTempvector=*it;
                    cout << myTempvector[0]<< " myTempvector[0]"<<endl;
                    cout << modelName<< " modelName"<<endl;
                    if (myTempvector[0]==modelName && foundIndex==0){
                        lastFatherIndex=fathers[modelName];
                        foundIndex=1;
                        cout << lastFatherIndex << " lastFatherIndex"<<endl;
                        
                    }
                }
                if (foundIndex==0){
                    countFathers+=1;
                    fathers[modelName] = countFathers;  //

                    cout << modelName<< " dad added2"<<endl;
                    cout << countFathers<< " countFathers"<<endl;
                    vector<string> tempVector;

                    tempVector.push_back(modelName);
                    adjacencyList.push_back(tempVector);
                    visitedLocations.push_back(modelName);
                   
                }
            }else{
                countFathers+=1;
                fathers[modelName] = countFathers;  //

                cout << modelName<< " dad added1"<<endl;
                 cout << countFathers<< " countFathers1"<<endl;
                vector<string> tempVector;
                tempVector.push_back(modelName);
                adjacencyList.push_back(tempVector);
                visitedLocations.push_back(modelName);
            }

        }else{
        	if (modelName.find("se1") !=string::npos ){ //find the word se1 in the modelName
	            if (find(visitedLocations.begin(), visitedLocations.end(), modelName) == visitedLocations.end()) { //if new model found

	            	cout << lastFatherIndex<< " lastFatherIndex3"<<endl;
	            	if (adjacencyList.size()>0){
	            		cout << adjacencyList.size()<< " adjacencyList.size()"<<endl;
	            		adjacencyList[lastFatherIndex].push_back(modelName);
	            		cout << " new model"<<endl;
	            		visitedLocations.push_back(modelName);
	            		for (vector< vector<string> >::iterator it=adjacencyList.begin(); it !=adjacencyList.end(); ++it){
	            			vector<string>  myTempvector=*it;
	            			for (vector<string>::iterator it2=myTempvector.begin(); it2 !=myTempvector.end(); ++it2){
	            				cout << *it2 << endl;
	            			}
	            		}
	            	}else{
	            		cout << "no father in adjacencyList" << endl;
	            	}
	            }
        	}
    	}	

    	*/

}
/*
int main()
{
    // Create a DirectionalGraph given in the above diagram
    DirectionalGraph g(4);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 0);
    g.addEdge(2, 3);
    g.addEdge(3, 3);
 
    cout << "Following is Depth First Traversal"
            " (starting from vertex 2) \n";
    g.DFS(2);
 
    return 0;
}
*/

void DirectionalGraph::readObjectsDetected(const create_topomap::logicalImage::ConstPtr& msg)
{
  //cout <<"I heard: "<<msg->modelName<<endl;
  	int direction[2]={};
	int compass;
	getDirection(direction);
	compass=direction[1];
	stringstream ss;
	ss << compass;
	string modelName=	msg->modelName+ss.str(); //model name plus compass reference

  createGraph(modelName,direction);
}


void DirectionalGraph::readDirection(const std_msgs::Float64::ConstPtr& msg)
{
  //cout <<"I heard: "<<msg->data<<endl;
  mydirection_robot=msg->data;
  
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "create_topomap_node");

  DirectionalGraph topomap;

 
  //ros::NodeHandle n;

  //ros::Subscriber sub = n.subscribe("/objectsDetected", 1000, readObjectsDetected);

  ros::spin();

  return 0;
}
