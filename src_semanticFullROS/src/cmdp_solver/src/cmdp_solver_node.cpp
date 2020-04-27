/*This service reads the 2 adjacency matrices (adjMatrixLeft,adjMatrixRight)
that represent the graph after the exploration. It creates the 
stateAction table that will describe the different transitions of the
whole graph. It fills this table with the transition probabilites, the
costs, and the actions that will describe how to move the robot.

Specifically we have a graph of the environment described by to matrices
and we transform this graph into a table for all the possible transitions
for each of the nodes in the graph. 

In this case each state is a node of the graph, so for example for state corridor2b,
when it moves with action crossDoor fast to left it means that the robot will end at the next
state room13 with probability 0.6.

After having the tables with the transitions information we proced to solve the CMDP and find a policy. 
We use the library glpk wiht the linprog function to solve the linear program while the constraints are respected.
see documentation: https://www.gnu.org/software/glpk/

note: The success probabilty for the states that fall into the obstacles(sink state) is 0.0
because it means failure, the same for the transition from the sink to the final.

Author: Jose Luis Susa Rincon 
September, 2018
Version 1.0
*/

#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <cstdlib>
#include <time.h>
#include <fstream>
#include <random>
#include <glpk.h>
#include "cmdp_solver/solveCMDP.h"
#include <pwd.h>

using namespace std;

class CMDP{

	public:
		string sharedFolder;
		vector <vector<int> > adjMatrixLeft,adjMatrixRight;
		vector<int> listNodes;
		vector<string>  listOfNodesType;
		vector<int> cdoorF,cdoorSl,corrF,corrSl;
		float tpCdoorF,tpCdoorSl,tpCorrF,tpCorrSl;

		//cmdp transitions =graph with edges, rewards and transition probabilities
        vector< int > statesTransitionTable;
        vector< int > actionsTransitionTable;
        vector< int > nextStatesTransitionTable;
        vector< float > costTransitionTable;
        vector< float > probTransitionTable;
        vector< vector<int> > indexPerStatesTransition,indexPerStatesTransition2; //this saves all the indexes where each state is found in the transition table

		//CMDP
    	vector< int > goalCells;
    	int start;
		float timeContraint,failureProb; //Constraints for the inequalities for the CMDP
   
		//Functions
		CMDP();
		int readAdjMatFile(vector <vector<int> > & ,ifstream& );
		int readListFile(vector <int> &,ifstream&);
		int readListTypeFile(vector <string> &,ifstream&);
		int readFiles(uint, uint,float, float);
		int findNeighborsData(vector <int> &,vector <int> &,vector <float> &,int );
};

CMDP::CMDP(){
	sharedFolder="";

}


//reads the adjanceny matrix file and fills a vector with the values 
//The matrix is built this way: adjMatrix=  [0 0 0 1;0 1 1 0;0 0 0 1;1 0 1 0]; 
int CMDP::readAdjMatFile(vector <vector<int> > & matrix,ifstream& file){
	vector<int> tempRow;

  	string line;
  	if (file.is_open())  {
        cout << "File opened with the adjancency matrices" << '\n';
        while ( getline (file,line) ){
			std::stringstream stream(line);
            while(1) {
				int n;
				stream >> n;
				if(!stream)
					break;
				tempRow.push_back(n);  
			}
			matrix.push_back(tempRow);
			tempRow.clear();
            //cout << line << '\n';
        }
        file.close();
  }else{
    cout << "Unable to open file \n"; 
    return -1; //error
  }

  return 1;
}

int CMDP::readListFile(vector <int> & list,ifstream& file){

  string line;
  if (file.is_open())  {
        cout << "File opened with the list of nodes" << '\n';
        while ( getline (file,line) ){
			stringstream stream(line);
			int n;
            stream >> n;
			list.push_back(n);
            
        }
        file.close();
  }else{
    cout << "Unable to open file \n"; 
    return -1; //error
  }

  return 1;
}

int CMDP::readListTypeFile(vector <string> & list,ifstream& file){

  string line;
  if (file.is_open())  {
        cout << "File opened with the list of nodes" << '\n';
        while ( getline (file,line) ){
			list.push_back(line);
            
        }
        file.close();
  }else{
    cout << "Unable to open file \n"; 
    return -1; //error
  }

  return 1;
}
int CMDP::readFiles(uint myStart, uint myTarget,float myTempDeadLine, float myFailProb){
	
	string listNodesNameFile=sharedFolder+"listNodes.txt";
	string listTypeNodesNameFile=sharedFolder+"listNodesType.txt";
	string adjMatrixLeftNameFile=sharedFolder+"adjMatrixLeft.txt";
	string adjMatrixRightNameFile=sharedFolder+"adjMatrixRight.txt";

	string cdoorFNameFile=sharedFolder+"doorFastLin6Ang6.txt";
	string cdoorSlFNameFile=sharedFolder+"doorSlowLin2Ang2.txt";
	string corrFFNameFile=sharedFolder+"corridorFastLin7Ang7.txt";
	string corrSlFNameFile=sharedFolder+"corridorSlowLin3Ang3.txt";

	//string goalCellsNameFile=sharedFolder+"target.txt";
	//string startNameFile=sharedFolder+"start.txt";

	//string tempDeadLineNameFile=sharedFolder+"tempDeadLine.txt";
	//string tempFailProbNameFile=sharedFolder+"tempFailProb.txt";

	ifstream listNodesFile (listNodesNameFile.c_str());
    ifstream listTypeNodesFile (listTypeNodesNameFile.c_str());
    ifstream adjMatrixLeftFile (adjMatrixLeftNameFile.c_str());
	ifstream adjMatrixRightFile (adjMatrixRightNameFile.c_str());

	ifstream cdoorFFile (cdoorFNameFile.c_str());
	ifstream cdoorSlFile (cdoorSlFNameFile.c_str());
	ifstream corrFFile (corrFFNameFile.c_str());
	ifstream corrSlFile (corrSlFNameFile.c_str());

	//ifstream goalCellsFile (goalCellsNameFile.c_str());
	//ifstream startFile (startNameFile.c_str());

	//ifstream tempDeadLineFile (tempDeadLineNameFile.c_str());
	//ifstream tempFailProbFile (tempFailProbNameFile.c_str());

	if(readListFile(listNodes,listNodesFile)==-1){
		cout<<"Error reading list nodes"<<endl;
		return -1;
	}

	if(readListTypeFile(listOfNodesType,listTypeNodesFile)==-1){
		cout<<"Error reading list type nodes"<<endl;
		return -1;
	}
	if(readAdjMatFile(adjMatrixLeft,adjMatrixLeftFile)==-1){
		cout<<"Error reading adjacency matrix left"<<endl;
		return -1;
	}
	if(readAdjMatFile(adjMatrixRight,adjMatrixRightFile)==-1){
		cout<<"Error reading adjacency matrix right"<<endl;
		return -1;
	}
	if(readListFile(cdoorF,cdoorFFile)==-1){
		cout<<"Error reading cdoorF"<<endl;
		return -1;
	}
	if(readListFile(cdoorSl,cdoorSlFile)==-1){
		cout<<"Error reading cdoorSl"<<endl;
		return -1;
	}
	if(readListFile(corrF,corrFFile)==-1){
		cout<<"Error reading corrF"<<endl;
		return -1;
	}
	if(readListFile(corrSl,corrSlFile)==-1){
		cout<<"Error reading corrSl"<<endl;
		return -1;
	}
	/*
	if(readListFile(goalCells,goalCellsFile)==-1){
		cout<<"Error reading goalCells"<<endl;
		return -1;
	}
	*/
	goalCells.push_back(myTarget);
 	/*
	string line;
  	if (startFile.is_open())  {
        cout << "File opened with start" << '\n';
        getline (startFile,line);
		stringstream stream(line);
		int n;
		stream >> n;
		start=n;      
        startFile.close();
	}else{
		cout << "Unable to open start file \n"; 
		return -1; //error
	}*/
	//start
	start=myStart;

	/*
	if (tempDeadLineFile.is_open())  {
        cout << "File opened with tempDeadLineFile" << '\n';
        getline (tempDeadLineFile,line);
		stringstream stream(line);
		float n;
		stream >> n;
		timeContraint=n;      
        tempDeadLineFile.close();
	}else{
		cout << "Unable to open tempDeadLineFile file \n"; 
		return -1; //error
	}
	*/
	timeContraint=myTempDeadLine;

 	/*
	if (tempFailProbFile.is_open())  {
        cout << "File opened with tempFailProbFile" << '\n';
        getline (tempFailProbFile,line);
		stringstream stream(line);
		float n;
		stream >> n;
		failureProb=n;      
        tempFailProbFile.close();
	}else{
		cout << "Unable to open tempFailProbFile file \n"; 
		return -1; //error
	}
 	*/
	failureProb=myFailProb;

	cout<<"Read from client: "<<start<<" "<<goalCells[0]<<" "<<timeContraint<<" "<<failureProb<<endl;
	return 1;
}

int CMDP::findNeighborsData(vector <int> &currentNeighborsY,vector <int> &currentActionsY, vector <float> &probabilitiesMat,int stateY){
	for(int i=0; i<statesTransitionTable.size();i++){
		if(statesTransitionTable[i]==stateY){
			currentNeighborsY.push_back(nextStatesTransitionTable[i]);
			currentActionsY.push_back(actionsTransitionTable[i]);
			probabilitiesMat.push_back(probTransitionTable[i]);
		}
	}
	return 1;
}

bool solvingCMDP(cmdp_solver::solveCMDP::Request  &req, cmdp_solver::solveCMDP::Response &res){
	if(req.startSolver==1)
		cout<<"Request received to solve CMDP"<<endl;

	CMDP myCMDP;
	 //get the home path if we want to save the files to the home folder
	const char *homedir;
	homedir=getenv("HOME");
	if ( homedir == NULL ) {
    	homedir = getpwuid(getuid())->pw_dir;
	}	
    string s1(homedir);
	myCMDP.sharedFolder=s1+"/sharedFolderLinuxSemantic/";
	//myCMDP.sharedFolder="/media/sharedFolderLinuxSemantic/"; //you can define your own folder to share the files

	// *********** reads the adjancency matrix
    // matlab function --> [sinkState,finalState,stateAction,numberActionsLeft,numberActionsRight]=adjMatrixToStateAction(goalCells);

	myCMDP.readFiles(req.start,req.target,req.tempDeadLine,req.failProb);

	//********** adjust indexes from matlab to ros discounting 1
	myCMDP.start=myCMDP.start-1;
	for(int i=0;i<myCMDP.listNodes.size();i++){
		myCMDP.listNodes[i]=myCMDP.listNodes[i]-1;
	}
	for(int i=0;i<myCMDP.goalCells.size();i++){
		myCMDP.goalCells[i]=myCMDP.goalCells[i]-1;
	}

	//********

	vector<int> listOfNodesIndex;
	for(int i=0;i<myCMDP.listNodes.size();i++){
		listOfNodesIndex.push_back(i);
	}

	//corridor to corridor action is stayCenter(1)
    //corridor to room action is crossdoor(2)
	vector<int> actionsPerNode;
	for(int i=0;i<myCMDP.listNodes.size();i++){
		actionsPerNode.push_back(0);
	}
	for(int i=0;i<myCMDP.listOfNodesType.size();i++){
		if(myCMDP.listOfNodesType[i]=="c")
			actionsPerNode[i]=0; //found a corridor so stay center
		else
			actionsPerNode[i]=1; //found a room then the action is cross door
	}

	// ********* Table 1:********
    //staycenter left (Fast),  crossdoor left(Fast)
    //staycenter left (Slow),  crossdoor left(Slow)
    //stayCenter right(Fast), crossdoor right(Fast)
    //stayCenter right(Slow), crossdoor right(Slow)
    //speed  maneuver
    //   0       0       fast stayCenter
    //   0       1       fast crossdoor
    //   2       0       slow stayCenter
    //   2       1       slow crossdoor
	//

	vector < vector<int> >  tableActions={{0, 0},{0,1},{2,0},{2,1}};
    vector <int> speeds={0,2};//fast and slow in terms of time, 1 is less time because its fast
    vector <float> tableCostsPerAction={4.8378,13.8409,9.1266,36.6746}; //fastCorr, fastDoor, slowCorr, slowDoor
	


	//because for each action there is only one nextstate, and there are not multiple
    //tansition probabilities for any specific action, we can define a
    //different maneuver ID for each combination state + action + speed that will use the CMDP.
    
    //there are 4 different real maneuvers according to table 1 and there is a maximum of #states*#states possible IDs 
	vector < vector <int> > numberActionsLeft;
	vector <int> rowTemp;
	for(int i=0; i<4;i++){
		numberActionsLeft.push_back(rowTemp);
		for(int j=0; j<myCMDP.adjMatrixLeft.size()*myCMDP.adjMatrixLeft.size();j++){
			numberActionsLeft[i].push_back(0);
		}
		rowTemp.clear();
	}
    int countActions=1;

	for(int i=0; i<4;i++){
        for(int j=0; j<myCMDP.adjMatrixLeft.size()*myCMDP.adjMatrixLeft.size();j++){
            numberActionsLeft[i][j]=countActions;
            countActions++;       
        }
	}

	vector < vector <int> > numberActionsRight;
	for(int i=0; i<4;i++){
		numberActionsRight.push_back(rowTemp);
		for(int j=0; j<myCMDP.adjMatrixRight.size()*myCMDP.adjMatrixRight.size();j++){
			numberActionsRight[i].push_back(0);
		}
		rowTemp.clear();
	}

	for(int i=0; i<4;i++){
        for(int j=0; j<myCMDP.adjMatrixRight.size()*myCMDP.adjMatrixRight.size();j++){
            numberActionsRight[i][j]=countActions;
            countActions++;       
        }
	}

	//calculate transition probabilities
	int counterOnes=0;
	for(int i=0; i<myCMDP.cdoorF.size();i++){
		if(myCMDP.cdoorF[i]==1)
			counterOnes++;
	}
	myCMDP.tpCdoorF=(float)counterOnes/myCMDP.cdoorF.size();

	counterOnes=0;
	for(int i=0; i<myCMDP.cdoorSl.size();i++){
		if(myCMDP.cdoorSl[i]==1)
			counterOnes++;
	}
	myCMDP.tpCdoorSl=(float)counterOnes/myCMDP.cdoorSl.size();

	counterOnes=0;
	for(int i=0; i<myCMDP.corrF.size();i++){
		if(myCMDP.corrF[i]==1)
			counterOnes++;
	}
	myCMDP.tpCorrF=(float)counterOnes/myCMDP.corrF.size();

	counterOnes=0;
	for(int i=0; i<myCMDP.corrSl.size();i++){
		if(myCMDP.corrSl[i]==1)
			counterOnes++;
	}
	myCMDP.tpCorrSl=(float)counterOnes/myCMDP.corrSl.size();

	vector <vector <float> > transProbPerAction={{myCMDP.tpCorrF,myCMDP.tpCorrSl},{myCMDP.tpCdoorF,myCMDP.tpCdoorSl}};

	int numberStates = myCMDP.adjMatrixLeft.size()*2;

	//definition of sink State = number of States+1
    int sinkState=numberStates +1 ;

    //definition of final State = number of States+2 (sink +1)
    int finalState=numberStates+2;

    numberStates=numberStates+1;

	int numberRowsStateAction=numberStates*numberStates*2;


	int actionToUse=-1;

	int indexCost=-1;
	
	bool goalAdded= false;

	for(int i=0; i<myCMDP.adjMatrixLeft.size();i++){
		for(int j=0; j<myCMDP.adjMatrixLeft[i].size();j++){
			for(int k=0; k<myCMDP.goalCells.size();k++){
				if (myCMDP.adjMatrixLeft[i][j]==1 && listOfNodesIndex[i]!=myCMDP.goalCells[k]){
					for(int h=0; h<speeds.size();h++){
						myCMDP.statesTransitionTable.push_back(listOfNodesIndex[i]);
						myCMDP.nextStatesTransitionTable.push_back(listOfNodesIndex[j]);
                        if(actionsPerNode[listOfNodesIndex[i]]==0 && actionsPerNode[listOfNodesIndex[j]]==0){ //if the initial state is a corridor and the next state also a corridor use action go corridor
                            actionToUse=0; //if there is a movemennt between corridor to/from corridor then use action go corridor
						}else{
                            actionToUse=1; //if there is a movemennt between corridor to/from room then use action cross door
                        }
                        if (speeds[h]==0 && actionToUse==0){
                            indexCost=0;
						}else if (speeds[h]==2 && actionToUse==0){
                            indexCost=2;
						}else if (speeds[h]==0 && actionToUse==1){
                            indexCost=1;
						}else if (speeds[h]==2 && actionToUse==1){
                            indexCost=3;
                        }
                        myCMDP.costTransitionTable.push_back(tableCostsPerAction[indexCost]); //speed fast or slow
						myCMDP.probTransitionTable.push_back(transProbPerAction[actionToUse][h]);         
                        int indexAction=-1;
						for(int p=0;p<tableActions.size();p++){
							if(tableActions[p][0]==speeds[h] && tableActions[p][1]==actionToUse){
								indexAction=p;
							}
						}
                        myCMDP.actionsTransitionTable.push_back(numberActionsLeft[indexAction][j+(i)*myCMDP.adjMatrixLeft.size()]);
                        //countStateAction=countStateAction+1;
                    }
				}else{
					if(myCMDP.adjMatrixLeft[i][j]==1 && listOfNodesIndex[i]==myCMDP.goalCells[k] && goalAdded==false){
						myCMDP.statesTransitionTable.push_back(myCMDP.goalCells[k]); 
						myCMDP.nextStatesTransitionTable.push_back(finalState);
						myCMDP.costTransitionTable.push_back(0); //speed fast or slow
						myCMDP.probTransitionTable.push_back(1);  
						myCMDP.actionsTransitionTable.push_back(0);
						//countStateAction=countStateAction+1;
						goalAdded=true;
					}
				}

				if (myCMDP.adjMatrixRight[i][j]==1 && listOfNodesIndex[i]!=myCMDP.goalCells[k]){
					for(int h=0; h<speeds.size();h++){
						myCMDP.statesTransitionTable.push_back(listOfNodesIndex[i]);
						myCMDP.nextStatesTransitionTable.push_back(listOfNodesIndex[j]);
                        if(actionsPerNode[listOfNodesIndex[i]]==0 && actionsPerNode[listOfNodesIndex[j]]==0){ //if the initial state is a corridor and the next state also a corridor use action go corridor
                            actionToUse=0; //if there is a movemennt between corridor to/from corridor then use action go corridor
						}else{
                            actionToUse=1; //if there is a movemennt between corridor to/from room then use action cross door
                        }
                        if (speeds[h]==0 && actionToUse==0){
                            indexCost=0;
						}else if (speeds[h]==2 && actionToUse==0){
                            indexCost=2;
						}else if (speeds[h]==0 && actionToUse==1){
                            indexCost=1;
						}else if (speeds[h]==2 && actionToUse==1){
                            indexCost=3;
                        }
                        myCMDP.costTransitionTable.push_back(tableCostsPerAction[indexCost]); //speed fast or slow
						myCMDP.probTransitionTable.push_back(transProbPerAction[actionToUse][h]);         
                        int indexAction=-1;
						for(int p=0;p<tableActions.size();p++){
							if(tableActions[p][0]==speeds[h] && tableActions[p][1]==actionToUse){
								indexAction=p;
							}
						}
                        myCMDP.actionsTransitionTable.push_back(numberActionsRight[indexAction][j+(i)*myCMDP.adjMatrixRight.size()]);
                        //countStateAction=countStateAction+1;
                    }
				}else{
					if(myCMDP.adjMatrixRight[i][j]==1 && listOfNodesIndex[i]==myCMDP.goalCells[k] && goalAdded==false){
						myCMDP.statesTransitionTable.push_back(myCMDP.goalCells[k]); 
						myCMDP.nextStatesTransitionTable.push_back(finalState);
						myCMDP.costTransitionTable.push_back(0); //speed fast or slow
						myCMDP.probTransitionTable.push_back(1);  
						myCMDP.actionsTransitionTable.push_back(0);
						//countStateAction=countStateAction+1;
						goalAdded=true;
					}
				}
			}
		}
	}

	//sink state to final
	myCMDP.statesTransitionTable.push_back(sinkState); 
	myCMDP.nextStatesTransitionTable.push_back(finalState);
	myCMDP.costTransitionTable.push_back(0); //speed fast or slow
	myCMDP.probTransitionTable.push_back(1);  
	myCMDP.actionsTransitionTable.push_back(0);

	// *********** end adjMatrixToStateAction


	// ************  This function solves a CMDP problem with any number of constraints. 
	// matlab function --> [policyCMDP,rho,stateAction1]=solveCMDP7(mdp,constraints,typeOfBetha,start,sinkState,epsilon,MaximizationOrMinimization,observationAction)
  
	cout<<"Solving Linear Programing for CMDP"<<endl;
    
    int goFinalStateAction=9999;

	//filling valueSetCosts, first cost is the reward when getting to finalstate, second cost is the failure probability which is 1 only for the sink state, third cost is the time. 
	vector <vector <float> > valueSetCosts;
	vector <float> tempVectFloat;
	int target=myCMDP.goalCells[0];
	for(int i=0;i<myCMDP.statesTransitionTable.size();i++){
		if(myCMDP.nextStatesTransitionTable[i]==finalState && myCMDP.statesTransitionTable[i]==target)
			tempVectFloat.push_back(1);
		else
			tempVectFloat.push_back(0);

		if(myCMDP.statesTransitionTable[i]==sinkState)
			tempVectFloat.push_back(1);
		else
			tempVectFloat.push_back(0);
		
		if(myCMDP.nextStatesTransitionTable[i]==finalState || myCMDP.statesTransitionTable[i]==sinkState)
			tempVectFloat.push_back(0);
		else
			tempVectFloat.push_back(myCMDP.costTransitionTable[i]);

		valueSetCosts.push_back(tempVectFloat);
		//cout<<tempVectFloat[0]<<" "<<tempVectFloat[1]<<" "<<tempVectFloat[2]<<endl;
		tempVectFloat.clear();	
	}

	//F
	int numCostsEachState=valueSetCosts[0].size();
	vector <float> f;
	for(int i=0;i<valueSetCosts.size();i++){
		f.push_back(valueSetCosts[i][0]*-1);  //%if we are maximizing the objective function has to be multiply by -1
	}

	//A
	vector < vector<float> > A;
	tempVectFloat.clear();
	if (numCostsEachState>=2){
		for(int i=0;i<valueSetCosts.size();i++){
			tempVectFloat.push_back(valueSetCosts[i][1]);
		}
		A.push_back(tempVectFloat);
		tempVectFloat.clear();
		for(int i=0;i<valueSetCosts.size();i++){
			tempVectFloat.push_back(valueSetCosts[i][2]);
		}
		A.push_back(tempVectFloat);
	}else{
		cout<<"ERROR: this CMDP doesn't have constraints (no more than 1 cost per state), solve it with regular MDP"<<endl;
		return 0;
	}

	//b
	vector <float> b;
	b.push_back(myCMDP.failureProb);
	b.push_back(myCMDP.timeContraint);


	//Aeq
	int indexColumn=-1;
	int stateX=-1;
	int stateY=-1;
	vector <int> currentNeighborsY,currentActionsY;
	vector <float> probabilitiesMat;
	vector <float> probCurrentNeigh;
	vector <int>  currentActionsYNeigh;
	int dxy=-1;
	int currentAction=-1;

	float Pyx;
	vector<int> indxNeigh,inxActionsFound;
	int inxNeigFound;
	
	vector <int> statesVectorNoAbsor=myCMDP.listNodes;
	statesVectorNoAbsor.push_back(sinkState);


	//beq
	vector <float> beq;
	for(int i=0;i<statesVectorNoAbsor.size();i++){
		if(statesVectorNoAbsor[i]==myCMDP.start)
			beq.push_back(1);
		else
			beq.push_back(0);
	}

	vector < vector <float> > Aeq;
	tempVectFloat.clear();
	for(int i=0;i<statesVectorNoAbsor.size();i++){	//initialize Aeq
		for(int j=0;j<f.size();j++){
			tempVectFloat.push_back(0);
		}
		Aeq.push_back(tempVectFloat);
		tempVectFloat.clear();
	}

	for(int x=0;x<statesVectorNoAbsor.size();x++){	 //states in X
		indexColumn=0;
		stateX=statesVectorNoAbsor[x];
		for(int y=0;y<statesVectorNoAbsor.size();y++){	 //states in Y
			stateY=statesVectorNoAbsor[y];
			myCMDP.findNeighborsData(currentNeighborsY,currentActionsY,probabilitiesMat,stateY);
			indxNeigh.clear(); //index neighbors
			for(int i=0;i<currentNeighborsY.size();i++){
				if(currentNeighborsY[i]==stateX){
					indxNeigh.push_back(i);
				}
			}
			if(indxNeigh.size()>0){
				currentActionsYNeigh.clear();
				for(int i=0;i<indxNeigh.size();i++){
					probCurrentNeigh.push_back(probabilitiesMat[indxNeigh[i]]);
					currentActionsYNeigh.push_back(currentActionsY[indxNeigh[i]]);
				}
			}
			//else{
			//		cout<<"probCurrentNeigh and currentActionsYNeigh are empty"<<endl;		
			//}

			//dxy is 1 if x==y, 0 otherwise
			dxy=-1;
            if (stateX==stateY)
                dxy=1;
            else
                dxy=0;
            
			if(indxNeigh.size()==0 && stateX!=sinkState ){ 	//no neighbors found
				if(dxy==1){
					for(int i=0;i<currentActionsY.size();i++){
						Aeq[x][indexColumn+i]=dxy;
					}
					indexColumn=indexColumn+currentActionsY.size();
				}else{
					indexColumn=indexColumn+currentActionsY.size();
				}
			}else{
				for(int indxAction=0;indxAction<currentActionsY.size();indxAction++){
					currentAction=currentActionsY[indxAction];
					vector <int> currentActsYNeighTemp;
					for(int i=0;i<currentActionsYNeigh.size();i++){
						if(currentActionsYNeigh[i]==currentAction){
							currentActsYNeighTemp.push_back(i);
						}
					}
					if(currentActsYNeighTemp.size()==0 && stateX!=sinkState){
						Aeq[x][indexColumn]=dxy-0;
						indexColumn=indexColumn+1;
					}else{
						inxActionsFound.clear();
						for(int i=0;i<currentActionsY.size();i++){
							if(currentActionsY[i]==currentAction){
								inxActionsFound.push_back(i);
							}
						}
						inxNeigFound=0;
						for(int i=0;i<inxActionsFound.size();i++){
							if(currentNeighborsY[inxActionsFound[i]]==stateX){
								inxNeigFound=i;
							}
						}
						
						if(stateX==sinkState && stateY!=sinkState){ //each state can go to the sink state, this will have a probability of failure
							if(inxNeigFound==0){
								Pyx=0;
							}else{
								Pyx=probabilitiesMat[inxActionsFound[inxNeigFound]];
							}
						}else{
							if(stateX==sinkState && (stateY==sinkState || currentAction==goFinalStateAction)){
								Pyx=0;
							}else{
								Pyx=probabilitiesMat[inxActionsFound[inxNeigFound]];
							}
						}
						Aeq[x][indexColumn]=dxy-Pyx;
                        indexColumn=indexColumn+1;  
					}
				}
			}

			currentNeighborsY.clear();
			currentActionsY.clear();
			probabilitiesMat.clear();
		}
	}

	cout<<"Solving linprog..."<<endl;

	//Examples of small linear programs to test linprog:
	//vector <float> f1 = {10,6,4};
	//vector < vector<float> > A1= {{1,1,1},{10,4,5},{2,2,6}};
	//vector <int> b1 = {100,600,300};


	//vector <float> f1 = {-150,-175};
	//vector < vector<float> > A1= {{7,11},{10,8}};
	//vector <int> b1 = {77,80};
	//vector <vector<float>  > Aeq1 = {{12,1},{3,1}};
	//vector <float> beq1 = {2,1};


	glp_prob *lp;
	int sizeVector=(Aeq.size()*Aeq[0].size()+A.size()*A[0].size())*2; //the size of ia and ja must be bigger than the number of elements to store 
    int ia[1+sizeVector], ja[1+sizeVector];
    double ar[1+sizeVector], fval;
	s1:   lp = glp_create_prob();						//s1 creates a problem object
	s2:   glp_set_prob_name(lp, "cmdp_semantic");		//s2 assigns a symbolic name to the problem object
	s3:   glp_set_obj_dir(lp, GLP_MAX);					//s3 calls the routine glp_set_obj_dir in order to set the optimization direction flag, where GLP_MAX means maximization
	s4:   glp_add_rows(lp, f.size()+beq.size());					// s4 adds  rows to the problem object.
	s5:   glp_set_row_name(lp, 1, "p");					//s5 assigns the symbolic name ‘p’ to the first row
	s6:   glp_set_row_bnds(lp, 1, GLP_UP, 0.0, b[0]);	//s6 sets the type and bounds of the first row, where GLP_UP means that the row has an upper bound.
	s7:   glp_set_row_name(lp, 2, "q");
	s8:   glp_set_row_bnds(lp, 2, GLP_UP, 0.0, b[1]);

	 
	//beq
	for(int i=0;i<beq.size();i++){
		stringstream ss;
		ss << i;
		string name = 'b'+ss.str();
		const char* cstr2 = name.c_str();
		glp_set_row_name(lp, 3+i, cstr2);
	    glp_set_row_bnds(lp, 3+i, GLP_FX, beq[i], beq[i]);
	}

	s11:  glp_add_cols(lp, f.size());					//s11 adds  columns to the problem object.


	//objective coefficient
	for(int i=0;i<f.size();i++){
		stringstream ss;
		ss << i;
		string name = 'x'+ss.str();
		const char* cstr2 = name.c_str();
		s12:  glp_set_col_name(lp, 1+i, cstr2);				//s12 assigns the symbolic name ‘x1’ to the first column,the statement 
		s13:  glp_set_col_bnds(lp, 1+i, GLP_LO, 0.0, 0.0); 	//s13 sets the type and bounds of the first column, where GLP_LO means that the column has an lower bound
		s14:  glp_set_obj_coef(lp, 1+i, f[i]*-1);				//s14 sets the objective coefficient for the first column
	}

	int counter=1;
	for(int i=0;i<A.size();i++){
		for(int j=0;j<A[i].size();j++){
			s21:  ia[counter] = 1+i, ja[counter] = 1+j, ar[counter] =  A[i][j]; 		//s21—s29 prepare non-zero elements of the constraint matrix (i.e. constraint coefficients)
			counter++;
		}
	}

	for(int i=0;i<Aeq.size();i++){
		for(int j=0;j<Aeq[i].size();j++){
			ia[counter] = 1+A.size()+i, ja[counter] = 1+j, ar[counter] =  Aeq[i][j]; 		//s21—s29 prepare non-zero elements of the constraint matrix (i.e. constraint coefficients)
			counter++;		
		}
	}	  
	
	int checkDuplicates= glp_check_dup(Aeq.size()+A.size(), Aeq[0].size(), counter-1,ia,ja);

	if(checkDuplicates!=0){
		res.flagFinish = -2;
		cout<<"ERROR: checkDuplicates. "<<endl;
		return true;
	}
	s30:  glp_load_matrix(lp, counter-1, ia, ja, ar); 	//s30 calls the routine glp_load_matrix, which loads information from these three arrays into the problem object.
	
	
	glp_smcp parm;
	glp_init_smcp(&parm);
	parm.meth = GLP_DUAL; 						//dual simplex
	s31:  int flagCMDPSolver=glp_simplex(lp,  &parm);				// s31 calls the routine glp_simplex, which is a driver to the simplex method,
												// in order to solve the LP problem. This routine finds an optimal solution
												// and stores all relevant information back into the problem object.
	s32:  fval = glp_get_obj_val(lp);				//s32 obtains a computed value of the objective function
	//s33:  x1 = glp_get_col_prim(lp, 1);			//s33—s35 obtain computed values of structural variables(columns),
												// which correspond to the optimal basic solution found by the solver.
	cout<<"fval: "<<fval<<" "<<endl;
	
	cout<<"flagCMDPSolver: "<<flagCMDPSolver<<" "<<endl;

	//Saving files with policy
	ofstream statesPolFile,actionsPolFile, nextStatePolPolicy;

	string statesPolNameFile=myCMDP.sharedFolder+"statesPolicy.txt";
	string actionsPolNameFile=myCMDP.sharedFolder+"actionsPolicy.txt";
	string nextStatePolNameFile=myCMDP.sharedFolder+"nextStatesPolicy.txt";
	
	statesPolFile.open(statesPolNameFile.c_str());
	actionsPolFile.open(actionsPolNameFile.c_str());
	nextStatePolPolicy.open(nextStatePolNameFile.c_str());

	if(flagCMDPSolver==0){
		cout<<"A policy was found"<<endl;
		vector<double> rho, policyCMDP;
		//definition of epsilon, the number that decides if a value of rho can be considered enough to include it in the policy.
		double epsilon=1*pow(10,-10);

		for(int i=0;i<f.size();i++){
			//s36:  cout<<glp_get_col_prim(lp, 1+i)<<" ";
			cout<<glp_get_col_name(lp, 1+i)<<": "<<glp_get_col_prim(lp, 1+i)<<endl;
			rho.push_back(glp_get_col_prim(lp, 1+i));
		}
		s37:  glp_delete_prob(lp);

		//calculating policy from rho
		double sumRho;
		for(int rhoCount=0;rhoCount<rho.size();rhoCount++){
			vector<int> indexStateActions;
			//for(int r=0;r<myCMDP.statesTransitionTable.size();r++){
				for(int s=0;s<myCMDP.statesTransitionTable.size();s++){
					if(myCMDP.statesTransitionTable[rhoCount]==myCMDP.statesTransitionTable[s]){
						indexStateActions.push_back(s);
					}
				}		
			//}
			sumRho=0;
			for(int s=0;s<indexStateActions.size();s++){
				sumRho=sumRho+rho[indexStateActions[s]];
			}

			if(sumRho>epsilon)
				policyCMDP.push_back(rho[rhoCount]/sumRho);
			else
				policyCMDP.push_back(0);
			
			cout<<policyCMDP[policyCMDP.size()-1]<<" ";
		}
		
		cout<<endl<<"Saving policy files"<<endl;
		int currentState=-1;
		for(int i=0;i<myCMDP.listNodes.size();i++){
			currentState=myCMDP.listNodes[i];

			//cumsum policy
			vector<float> cumuSumProb,p;
			vector<int> inxP;  
			for(int j=0; j<myCMDP.statesTransitionTable.size(); j++){
				if(myCMDP.statesTransitionTable[j]==currentState){
					p.push_back(policyCMDP[j]);
					inxP.push_back(j);
				}
			}
			cout<<"cumsum"<<endl;
	        cumuSumProb.push_back(p[0]);
    	    for(int j=1; j<p.size(); j++){
        	    cumuSumProb.push_back(cumuSumProb[cumuSumProb.size()-1]+p[j]);
        	}
			float r = (rand()% 100)/100.0;
			int indexNextStateChoosen;
			for(indexNextStateChoosen=0;indexNextStateChoosen<cumuSumProb.size();indexNextStateChoosen++){
				if(cumuSumProb[indexNextStateChoosen]>r){
					break;
				}
			}
			int actionChosen;
			if(indexNextStateChoosen>=cumuSumProb.size()){ //for states where the policy is 0
				actionChosen=0;
			}else{
				actionChosen=myCMDP.actionsTransitionTable[inxP[indexNextStateChoosen]];
			}
			
			int finalAction=0;

			if(actionChosen>0 && actionChosen<pow(myCMDP.listNodes.size(),2)){
				finalAction=1;
			}else if(actionChosen>=pow(myCMDP.listNodes.size(),2) && actionChosen<pow(myCMDP.listNodes.size(),2)*2){
				finalAction=2;
			}else if(actionChosen>=pow(myCMDP.listNodes.size(),2)*2 && actionChosen<pow(myCMDP.listNodes.size(),2)*3){
				finalAction=3;
			}else if(actionChosen>=pow(myCMDP.listNodes.size(),2)*3 && actionChosen<pow(myCMDP.listNodes.size(),2)*4){
				finalAction=4;
			}else if(actionChosen>=pow(myCMDP.listNodes.size(),2)*4 && actionChosen<pow(myCMDP.listNodes.size(),2)*5){
				finalAction=5;
			}else if(actionChosen>=pow(myCMDP.listNodes.size(),2)*5 && actionChosen<pow(myCMDP.listNodes.size(),2)*6){
				finalAction=6;
			}else if(actionChosen>=pow(myCMDP.listNodes.size(),2)*6 && actionChosen<pow(myCMDP.listNodes.size(),2)*7){
				finalAction=7;
			}else if(actionChosen>=pow(myCMDP.listNodes.size(),2)*7 && actionChosen<pow(myCMDP.listNodes.size(),2)*8){
				finalAction=8;
			}

			int nextStateTemp=0;
			if(actionChosen!=0)
				nextStateTemp= myCMDP.nextStatesTransitionTable[inxP[indexNextStateChoosen]]; 
			else
				nextStateTemp=0;

			cout<<"state, action, next state: "<<myCMDP.listNodes[i]+1<<" "<<finalAction<<" "<<nextStateTemp+1<<endl;
			nextStatePolPolicy << nextStateTemp+1<<endl;
			statesPolFile << myCMDP.listNodes[i]+1<<endl; 
			actionsPolFile << finalAction <<endl; 
			res.flagFinish = 1;	   
		}
	}else {
		cout<<"A policy was NOT found"<<endl;
		for(int i=0;i<myCMDP.listNodes.size();i++){
			statesPolFile << myCMDP.listNodes[i]+1<<endl; 
			actionsPolFile << -1 <<endl; 
			nextStatePolPolicy << -1 <<endl; 
		}
		res.flagFinish = -1;
	}

	statesPolFile.close();
	actionsPolFile.close();
	nextStatePolPolicy.close();

  

  return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "cmdp_solver_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("solve_cmdp", solvingCMDP);
  	cout<<"Ready to add solve a cmdp."<<endl;
  	ros::spin();

	
	
  return 1;
}

