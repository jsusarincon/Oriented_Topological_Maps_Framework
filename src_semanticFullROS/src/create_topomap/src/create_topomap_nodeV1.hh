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
using namespace std;

class DirectionalGraph{

public:  
    //For the graph to be copied to a class
    vector< vector<string> > adjacencyList2Left,adjacencyList2Right;
    vector<string> visitedLocations;
    //map<string,int> fathers;   //each new node is a new father and we save the name of the father and the index on the adjencyList
    //int countFathers;
    int lastFatherIndex;
    int lastFathersDirection;
    string lastFatherName;
    int LEFTNUMBER;
    int RIGHTNUMBER;
    float mydirection_robot;
    bool getDirectionManually; //0 if we want automatic direction obtained from odom, 1 for manual human direction
    //end graph

    ros::NodeHandle n_; 
    ros::Subscriber sub_objects;
    ros::Subscriber sub_direction;
    //int V;    // No. of vertices
 
    // Pointer to an array containing
    // adjacency lists
    //list<int> *adj;
 
    
public:
    DirectionalGraph();   // Constructor
 
    // function to add an node to DirectionalGraph
    int addNode(string,int);
    // function to add an edge to DirectionalGraph for new node
    int addEdge(string,int);

    // function to add an edge to DirectionalGraph for existing node
    void addEdge(string,int,int);

    //function that finds if certain node has a edge to another node
    bool findEdge(string, string);

    //find the index of the father specified by the name string
    int findFatherIndex(string, int);

    // function to print the whole graph
    void printGraph();

    //save into a text file the graph
    void saveListsToFile();

    //get direction for new place left or right from a human or from compass (odom)
    void getDirection(int *); 
 
    // DFS traversal of the vertices
    // reachable from v
    //void DFS(int v);

    // A recursive function used by DFS
    //void DFSUtil(int v, bool visited[]);


    //subscribes to the objects that the logical camera detects
    void readObjectsDetected(const create_topomap::logicalImage::ConstPtr&);

    //subscribes to the direction of the robot, which is an angle from 0 to pi and from -pi to 0 being the horizontal x axis the origin
    void readDirection(const std_msgs::Float64::ConstPtr&);

    //creates the directional graph
    void createGraph(string , int *);
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