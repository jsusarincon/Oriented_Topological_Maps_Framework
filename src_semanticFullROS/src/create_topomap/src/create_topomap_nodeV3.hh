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
        bool finishedScaningEdgesFlag;
        map<string,int> edges; //it saves the edges for one node, the name of the edge and the direction
        map<string,int> visitedEdges;
        map<string,int> connectedNodes;
};

class DirectionalGraph{

public:  
    //For the graph to be copied to a class
    vector< vector<string> > adjacencyList2Left,adjacencyList2Right;
    vector<NodeSemantic> visitedLocations;  
    vector<string> fatherChild; //fatherChild saves the last node seen and the current node
    bool updateFatherChild;
    //map<string,int> fathers;   //each new node is a new father and we save the name of the father and the index on the adjencyList
    //int countFathers;

    // int lastFatherIndex;
    // int lastFathersDirection;
    // string lastFatherName;
    int lastDirectionTaken; 
    int LEFTNUMBER;
    int RIGHTNUMBER;
    float mydirection_robot;
    bool getDirectionManually; //0 if we want automatic direction obtained from odom, 1 for manual human direction
    //end graph

    //Ros
    ros::NodeHandle n_; 
    ros::Subscriber sub_objects;
    ros::Subscriber sub_direction;
    ros::Subscriber sub_collision;
    ros::Publisher goCorridor;
    ros::Publisher goDoor;

     //Topic to publish
    ros::Publisher pub_cmd_vel;

    string modelName,intersectionsName;
    float model_pos_x,model_pos_y;

    //Movement Robot

    
public:
    DirectionalGraph();   // Constructor
 
    // function to add an node to DirectionalGraph
    int addNode(string,int *);
   
    // function to add an edge to DirectionalGraph for new node
    void addEdge(string,int *, int, int);

    //function that finds if certain node has a edge to another node
    bool findEdge(string, string);

    //find the index of the father specified by the name string
    int findFatherIndex(string);

    //Updates the information of the last node seen for the robot which must be the last father that will host the next node
    void updateFather(string, int *);

    // function to print the whole graph
    void printGraph();

    //function to print the visited edges
    void  printVisitedLocations();

    //save into a text file the graph
    void saveListsToFile();

    //get direction for new place left or right from a human or from compass (odom)
    void getDirection(int *); 

    //we can send N,S,E,W and it will return if that is left 1 or right 2
    int getlOrR(int );
    
    //get diretion with wider angle to scan the intersections
    void getDirectionIntersections(int *);

    //creates the directional graph
    void createGraph(string , int *);

    int updateVisitedEdges(int*); 


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
    void scanForEdges(int, bool,int, string, int); //this function rotates the robot at the same place to find the edges of the current node (intersections)
    void faceNewDirection(int);
    int opositeDirection(int);
    void goNextEdge(NodeSemantic, int );
    void moveMiddleTag();

};

/*
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
  {
    PUBLISHED_MESSAGE_TYPE output;
    //.... do something with the input and generate the output...
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

*/

//#endif