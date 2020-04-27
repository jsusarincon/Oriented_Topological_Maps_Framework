//#ifndef _DIRECTIONAL_GRAPH_H_
//#define _DIRECTIONAL_GRAPH_H_

// DirectionalGraph class represents a directed DirectionalGraph
// using adjacency list representation

#include <iostream>
#include <list>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "create_topomap/logicalImage.h"
#include "create_topomap/collision.h"

using namespace std;

class NodeSemantic{
    public:
        string name;
        /*  1) East goes from -pi/4 to pi/4
            2) North goes from pi/4 to pi*3/4
            3) West goes from pi*3/4 to -pi*3/4
            4) South goes from -3/4*pi/4 to -pi/4
        */
        int direction;
        //float initScanEdgesAngle;
        bool finishedscanningEdgesFlag;
        map<string,int> edges; //it saves the edges for one node, the name of the edge and the direction
        map<string,int> visitedEdges; //name of the edge and if visite 1, unvisited 0
        map<string,int> connectedNodes; //name of the node and direction
        map<string,int> times; //it saves the time to go to each node using the system time
};

class DirectionalGraph{

public:  
    //For the graph to be copied to a class
    vector< vector<string> > adjacencyList2Left,adjacencyList2Right;
    vector<NodeSemantic> visitedLocations;  
    vector<string> fatherChild; //fatherChild saves the last node seen and the current node
    bool updateFatherChild;
    bool childFounded;
    //map<string,int> fathers;   //each new node is a new father and we save the name of the father and the index on the adjencyList
    //int countFathers;

    // int lastFatherIndex;
    // int lastFathersDirection;
    // string lastFatherName;
    int lastDirectionTaken; 
    int LEFTNUMBER;
    int RIGHTNUMBER;
    float mydirection_robot;
    float mydirection_robot_average[5];
    bool getDirectionManually; //0 if we want automatic direction obtained from odom, 1 for manual human direction
    int timeToNewNode; //the time spent using a maneuver and finding the new node
    //end graph

    //Ros
    ros::NodeHandle n_; 
    ros::Subscriber sub_objects;
    ros::Subscriber sub_direction;
    ros::Subscriber sub_collision;
    ros::Publisher goCorridor;
    ros::Publisher goDoor;
    bool collisionFlag;

     //Topic to publish
    ros::Publisher pub_cmd_vel;
    ros::ServiceClient clientSolveCMDP;

    string modelName,intersectionsName;
    float model_pos_x,model_pos_y;

    //testing
    int saveResultCounter; //When colliding only saves 1 time the failure. Without this variable the function collisionDetected is called many times and it would save all those times the failure ("0") in the file where we record the success or failure of the different experiments
    string sharedFolderName; //Shared Folder to comunicate with Matlab

    //Running time for experiments and calculate failure when temporalDeadline is not met
    int globalTimeStarts,globalTimeFinal; //saves the total time when using the manuevers
    int totalTimeToGoal;  //this value comes from the configuration file, dependeing of the experiment, the start and goal position changes and the average time spent to reach that goal from the starting point
    int timeLeftToGoal; //this value needs to be updated every time we call the CMDP and use certain amount of time to navigate, the CMDP receives "chunks" of the total time to navigate and get to the goal respecting the global total time (totalTimeToGoal).
    vector< float> slowCrossDoorTime, slowCorridorTime,fastCrossDoorTime, fastCorridorTime;

    
public:
    DirectionalGraph();   // Constructor
 
    // function to add an node to DirectionalGraph  using the visitedLocations
    int addNode2(string,int *);
   
    // function to add an edge to DirectionalGraph for new node using the visitedLocations
    int addEdge2(string,int *, int, int);

    //This function read the visitedLocations and build the adjacencyList2Left,adjacencyList2Right.
    int buildAdjancyMats();

    //function that finds if certain node has a edge to another node
    bool findEdge(string, string);

    //find the index of the father specified by the name string
    int findFatherIndex(string);

    //Updates the information of the last node seen for the robot which must be the last father that will host the next node
    void updateFather(string);

    // function to print the whole graph
    void printGraph();

    //function to print the visited edges
    void  printVisitedLocations();

    //save into a text file the graph
    int saveListsToFile();

    void endOfProgram(int);

    //get direction for new place left or right from a human or from compass (odom)
    int getDirection(int *); 

    //we can send N,S,E,W and it will return if that is left 1 or right 2
    int getlOrR(int );
    
    //get diretion with wider angle to scan the intersections
    int getDirectionIntersections(int *);

    //creates the directional graph
    void createGraph(string , int *);

    //This function updates the values for all the visited edges
    int updateVisitedEdges(int*); 

    //Exploration Strategies
    string randomExploration();
    string topologicalFrontier();
    string topologicalFrontierNormalized(string);
    string semanticExploration(string);
    string semanticExplorationRooms(string);

    //searching graph
    int dijkstra(int, int *, int, int,vector<int> &);
    int printSolution(int *, int );
    int minDistance(int, int *, bool*);
    // This function goes through the graph and find the path from a starting point to a goal point and select the temporal deadline according to the timeLeftToGoal
    float calcualteTemporalDeadLine(string ,string );
    
    // This function returns a path from 2 vertexes
    vector<int> getPath2Verx(string ,string );
    
    //Matlab
    int savingFilesForCMDP(float,float,string);

    //ROS
    //subscribes to the objects that the logical camera detects
    void readObjectsDetected(const create_topomap::logicalImage::ConstPtr&);

    //subscribes to the direction of the robot, which is an angle from 0 to pi and from -pi to 0 being the horizontal x axis the origin
    void readDirection(const std_msgs::Float64::ConstPtr&);

    // subscribes to collisionDetected which detect collisions with the robot
    void collisionDetected(const create_topomap::collision::ConstPtr&);
    
    //Move Robot
    void moveRobot(float,float);
    void stopRobot();
    int scanForEdges(int, bool,int, string, int); //this function rotates the robot at the same place to find the edges of the current node (intersections)
    string faceNewDirection(int);
    int opositeDirection(int);
    int goNextEdge(NodeSemantic, int );
    int moveMiddleTag();
    string goCorridorFunc(string,int);
    string goCrossDoorFunc(string,int);

};

