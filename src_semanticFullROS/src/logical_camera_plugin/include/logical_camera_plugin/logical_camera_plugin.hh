#ifndef _LOGICAL_CAMER_PLUGIN_HH_
#define _LOGICAL_CAMERA_PLUGIN_HH_

#include <string>
#include <vector>
#include <thread>
#include <unordered_map>

#include <sdf/sdf.hh>
#include "gazebo/math/Pose.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/logical_camera_image.pb.h"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

using namespace std;

namespace gazebo
{
  class LogicalCameraPlugin : public ModelPlugin
  {
    protected: double wheelSeperation;

    protected: double wheelRadius;

    public: enum {RIGHT_FRONT, RIGHT_REAR, LEFT_FRONT, LEFT_REAR};

    protected: int RegisterJoint(int _index, const std::string &_name);

    public: LogicalCameraPlugin();

    public: ~LogicalCameraPlugin();

    protected: physics::ModelPtr model;

    public: physics::WorldPtr world;

    protected: physics::LinkPtr cameraLink;

    protected: physics::JointPtr joint[4];

    protected: sensors::SensorPtr sensor;

    protected: std::string name;

    protected: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected: void findLogicalCamera();

    public: void onImage(ConstLogicalCameraImagePtr &_msg);

    protected: void publishTF( const math::Pose &pose, const std::string &parentFrame, const std::string &frame);

    public: void onRosLfMsg(const geometry_msgs::Twist::ConstPtr& _msg);

    public: void onRosRfMsg(const std_msgs::Float32ConstPtr &_msg);

    public: void onRosLrMsg(const std_msgs::Float32ConstPtr &_msg);
    
    public: void onRosRrMsg(const std_msgs::Float32ConstPtr &_msg);

    protected: void queueThread();

    protected: transport::NodePtr node;

    protected: transport::SubscriberPtr imageSub;

    protected: std::string robotNamespace;

    protected: ros::NodeHandle *rosnode;

    protected: ros::Publisher imagePub;

    protected: ros::Subscriber rosSubLF;

    protected: ros::Subscriber rosSubRF;

    protected: ros::Subscriber rosSubLR;

    protected: ros::Subscriber rosSubRR;

    protected: ros::CallbackQueue rosQueue;

    protected: std::thread rosQueueThread;

    protected: std::string modelFramePrefix;

    protected: boost::shared_ptr<tf::TransformBroadcaster> transformBroadcaster;

    protected: unordered_map<string,string> matchModel;

    protected: void initTable();

    protected: string checkIfRoom(string model_name);

  };
}

#endif
