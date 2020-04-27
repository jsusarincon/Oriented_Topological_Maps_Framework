#include <ros/ros.h>
#include <tf/tranform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
  ros::init(argc,argv,"p3at_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()) {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("p3at","p3at",ros::Time(0), transform
    }
  }
}

