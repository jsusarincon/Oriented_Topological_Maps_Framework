#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <cstdlib>
#include <time.h>
#include <fstream>
#include <random>
#include <pwd.h>

using namespace std;


int main(int argc, char **argv){

	ros::init(argc, argv, "testing_semantic_cmdp_node");

	 //get the home path if we want to save the files to the home folder
	const char *homedir;
	homedir=getenv("HOME");
	if ( homedir == NULL ) {
    	homedir = getpwuid(getuid())->pw_dir;
	}	
    string s1(homedir);
	string dataFileName=s1+"/sharedFolderLinuxSemantic/dataExperimentsExploration.txt";
	ifstream dataFile;
	dataFile.open (dataFileName, std::ifstream::in);

	vector< vector<int> > dataExperimentsTotal,successExperiments;
	vector<int> tempRow;

  	string line;
  	if (dataFile.is_open())  {
        cout << "File opened with the data experiments" << '\n';
        while ( getline (dataFile,line) ){
			std::stringstream stream(line);
            while(1) {
				int n;
				stream >> n;
				if(!stream)
					break;
				tempRow.push_back(n);  
			}
			dataExperimentsTotal.push_back(tempRow);
			tempRow.clear();
            //cout << line << '\n';
        }
        dataFile.close();
	}else{
		cout << "Unable to open file \n"; 
		return -1; //error
	}

	//680,477,272,68, 2899,2030,1160,290, 5628,3940,2251,563
	
	int timeSpent[12]={0};
	int numberDiscoveredVertices[12]={0};
	int counterSuccess[12]={0};
	int counterTries[12]={0};

	for(int i=0;i<dataExperimentsTotal.size();i++){
		tempRow.clear();
		//cout<<dataExperimentsTotal[i][0]<<" "<<dataExperimentsTotal[i][2]<<endl;
		if(dataExperimentsTotal[i][0]==1){
			for(int j=0;j<dataExperimentsTotal[i].size();j++){
				tempRow.push_back(dataExperimentsTotal[i][j]);
			}
			
			switch (dataExperimentsTotal[i][2]){
				case 680:
					timeSpent[0]=timeSpent[0]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[0]=numberDiscoveredVertices[0]+dataExperimentsTotal[i][5];
					counterSuccess[0]++;
					break;
				case 477:
					timeSpent[1]=timeSpent[1]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[1]=numberDiscoveredVertices[1]+dataExperimentsTotal[i][5];
					counterSuccess[1]++;
					break;
				case 272:
					timeSpent[2]=timeSpent[2]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[2]=numberDiscoveredVertices[2]+dataExperimentsTotal[i][5];
					counterSuccess[2]++;
					break;
				case 68:
					timeSpent[3]=timeSpent[3]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[3]=numberDiscoveredVertices[3]+dataExperimentsTotal[i][5];
					counterSuccess[3]++;
					break;
				case 2899:
					timeSpent[4]=timeSpent[4]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[4]=numberDiscoveredVertices[4]+dataExperimentsTotal[i][5];
					counterSuccess[4]++;
					break;
				case 2030:
					timeSpent[5]=timeSpent[5]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[5]=numberDiscoveredVertices[5]+dataExperimentsTotal[i][5];
					counterSuccess[5]++;
					break;
				case 1160:
					timeSpent[6]=timeSpent[6]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[6]=numberDiscoveredVertices[6]+dataExperimentsTotal[i][5];
					counterSuccess[6]++;
					break;
				case 290:
					timeSpent[7]=timeSpent[7]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[7]=numberDiscoveredVertices[7]+dataExperimentsTotal[i][5];
					counterSuccess[7]++;
					break;
				case 5628:
					timeSpent[8]=timeSpent[8]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[8]=numberDiscoveredVertices[8]+dataExperimentsTotal[i][5];
					counterSuccess[8]++;
					break;
				case 3940:
					timeSpent[9]=timeSpent[9]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[9]=numberDiscoveredVertices[9]+dataExperimentsTotal[i][5];
					counterSuccess[9]++;
					break;
				case 2251:
					timeSpent[10]=timeSpent[10]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[10]=numberDiscoveredVertices[10]+dataExperimentsTotal[i][5];
					counterSuccess[10]++;
					break;
				case 563:
					timeSpent[11]=timeSpent[11]+dataExperimentsTotal[i][1];
					numberDiscoveredVertices[11]=numberDiscoveredVertices[11]+dataExperimentsTotal[i][5];
					counterSuccess[11]++;
					break;
				default:
					cout<<"ERROR: Analysing data."<<endl;
					return -1;
			}
			
		

			successExperiments.push_back(tempRow);
		}

			switch (dataExperimentsTotal[i][2]){
				case 680:
					counterTries[0]++;
					break;
				case 477:
					counterTries[1]++;
					break;
				case 272:
					counterTries[2]++;
					break;
				case 68:
					counterTries[3]++;
					break;
				case 2899:
					counterTries[4]++;
					break;
				case 2030:
					counterTries[5]++;
					break;
				case 1160:
					counterTries[6]++;
					break;
				case 290:
					counterTries[7]++;
					break;
				case 5628:
					counterTries[8]++;
					break;
				case 3940:
					counterTries[9]++;
					break;
				case 2251:
					counterTries[10]++;
					break;
				case 563:
					counterTries[11]++;
					break;
				default:
					cout<<"ERROR: Analysing data."<<endl;
					return -1;
			}
		
	}
	char goals[12]={'A','A','A','A','B','B','B','B','C','C','C','C'};
	int tempDeadLines[12]={680,477,272,68,2899,2030,1160,290,5628,3940,2251,563};
	for(int i=0;i<12;i++){
		if(counterSuccess[i]>0)
			cout<<goals[i]<<" "<<tempDeadLines[i]<<" "<<timeSpent[i]/(float)counterSuccess[i]<<" "<<counterSuccess[i]/(float)counterTries[i]*100<<" "<<numberDiscoveredVertices[i]/(float)counterSuccess[i]<<endl;
		else
			cout<<goals[i]<<" "<<tempDeadLines[i]<<" "<<"NA"<<" "<<"NA"<<endl;

	}
	
  return 0;
}
