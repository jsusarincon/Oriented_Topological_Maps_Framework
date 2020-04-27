#include "logical_camera_plugin/logical_camera_plugin.hh"

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include "std_msgs/String.h"
#include "logical_camera_plugin/logicalImage.h"

#include <sstream>
#include <string>
#include <algorithm>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LogicalCameraPlugin);

LogicalCameraPlugin::LogicalCameraPlugin()
{
}

LogicalCameraPlugin::~LogicalCameraPlugin()
{
  this->rosnode->shutdown();
}

void LogicalCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  gzdbg << "Called Load!" << "\n";
  initTable();
  this->robotNamespace = "logical_camera";
  if(_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  this->world = _parent->GetWorld();
  this->name = _parent->GetName();

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
       << "unable to load plugin. Load the Gazebo system plugin "
       << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }


  this->modelFramePrefix = this->name + "_";
  if (_sdf->HasElement("model_frame_prefix"))
  {
    this->modelFramePrefix = _sdf->GetElement("model_frame_prefix")->Get<std::string>();
  }
  gzdbg << "Using model frame prefix of: " << this->modelFramePrefix << std::endl;

  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  this->findLogicalCamera();
  if ( !this->sensor)
  {
    gzerr << "No logical camera found on any link\n";
    return;
  }


  for (int i=0;i<4;i++){
    this->joint[i] = this->model->GetJoints()[i];
    gzdbg << "Joints found are: " <<this->joint[i]->GetScopedName() << "\n";
  }



  std::string imageTopic_ros = this->name;
  if (_sdf->HasElement("image_topic_ros")) 
  {
    imageTopic_ros = _sdf->Get<std::string>("image_topic_ros");
  }

  this->imageSub = this->node->Subscribe(this->sensor->Topic(),
  	&LogicalCameraPlugin::onImage, this);
  gzdbg << "Subscribing to gazebo topic: "<< this->sensor->Topic() << "\n";

  imagePub = this->rosnode->advertise<logical_camera_plugin::logicalImage>("/objectsDetected", 1000);
  gzdbg << "Publishing to ROS topic: " << imagePub.getTopic() << "\n";

  transformBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

}

void LogicalCameraPlugin::initTable()
{
  gzdbg << "Initializing hash table!" << "\n";

  //Rooms
  //convention is direction to enter the door + name of room
  // (WEST) side of the building
  this->matchModel.emplace("corridor_0_clone_0","West_se101");
  this->matchModel.emplace("corridor_0_clone_1","West_se102");
  this->matchModel.emplace("corridor_0_clone_14","East_se103");
  this->matchModel.emplace("corridor_0_clone_8","West_se104");
  this->matchModel.emplace("corridor_0_clone_9","West_se105");
  this->matchModel.emplace("corridor_0_clone_10","West_se106");
  this->matchModel.emplace("corridor_0_clone_11","West_se107");
  // (EAST) side of the building
  this->matchModel.emplace("corridor_0_clone_13","South_se108");
  this->matchModel.emplace("corridor_0_clone_12","West_se108");
  this->matchModel.emplace("corridor_0_clone_7","West_se109");
  this->matchModel.emplace("corridor_0_clone_6","West_se110");
  this->matchModel.emplace("corridor_0_clone_5","West_se111");
  this->matchModel.emplace("corridor_0_clone_4","West_se112");
  this->matchModel.emplace("corridor_0_clone_3","West_se113");
  this->matchModel.emplace("corridor_0_clone_2","West_se114");
  this->matchModel.emplace("corridor_22_clone","North_se115");
  this->matchModel.emplace("corridor_0_clone","West_se115");
  
  //edges rooms
  this->matchModel.emplace("intersection_58","intersection_East_101");
  this->matchModel.emplace("intersection_59","intersection_East_102");
  this->matchModel.emplace("intersection_60","intersection_West_103");
  this->matchModel.emplace("intersection_67","intersection_East_104");
  this->matchModel.emplace("intersection_68","intersection_East_105");
  this->matchModel.emplace("intersection_66","intersection_East_106");
  this->matchModel.emplace("intersection_69","intersection_East_107");

  this->matchModel.emplace("intersection_12_clone","intersection_North_108");
  this->matchModel.emplace("intersection_71","intersection_East_108");
  this->matchModel.emplace("intersection_33_clone_6","intersection_East_109");
  this->matchModel.emplace("intersection_24","intersection_East_110");
  this->matchModel.emplace("intersection_25","intersection_East_111");
  this->matchModel.emplace("intersection_63","intersection_East_112");
  this->matchModel.emplace("intersection_62","intersection_East_113");
  this->matchModel.emplace("intersection_42","intersection_East_114");
  this->matchModel.emplace("intersection_41","intersection_East_115");
  this->matchModel.emplace("intersection_40","intersection_South_115");


//intersections
  this->matchModel.emplace("intersection_32","intersection_East_001a");
  this->matchModel.emplace("intersection_31","intersection_West_001b");
  this->matchModel.emplace("intersection_29","intersection_East_001b");
  this->matchModel.emplace("intersection_37","intersection_North_001b");
  this->matchModel.emplace("intersection_28","intersection_West_001c");
  this->matchModel.emplace("intersection_30","intersection_East_001c");
  this->matchModel.emplace("intersection_0","intersection_North_001c");
  this->matchModel.emplace("intersection_34","intersection_West_001d");
  this->matchModel.emplace("intersection_2","intersection_North_001d");

  this->matchModel.emplace("intersection_38","intersection_South_002a");
  this->matchModel.emplace("intersection_56","intersection_North_002a");
  this->matchModel.emplace("intersection_3","intersection_West_002a");
  this->matchModel.emplace("intersection_46","intersection_South_002b");
  this->matchModel.emplace("intersection_9","intersection_East_002b");
  this->matchModel.emplace("intersection_45","intersection_North_002b");
  this->matchModel.emplace("intersection_45_clone","intersection_South_002c");
  this->matchModel.emplace("intersection_4","intersection_North_002c");
  this->matchModel.emplace("intersection_8","intersection_West_002c");
  this->matchModel.emplace("intersection_5","intersection_South_002d");
  this->matchModel.emplace("intersection_6","intersection_North_002d");
  this->matchModel.emplace("intersection_23","intersection_West_002d");
  this->matchModel.emplace("intersection_57","intersection_South_002e");
  this->matchModel.emplace("intersection_65","intersection_North_002e");
  this->matchModel.emplace("intersection_22","intersection_West_002e");
  this->matchModel.emplace("intersection_51","intersection_South_002f");
  this->matchModel.emplace("intersection_50","intersection_North_002f");
  this->matchModel.emplace("intersection_20","intersection_West_002f");
  this->matchModel.emplace("intersection_49","intersection_South_002g");
  this->matchModel.emplace("intersection_48","intersection_North_002g");
  this->matchModel.emplace("intersection_11","intersection_West_002g");

  this->matchModel.emplace("intersection_18","intersection_East_003a");
  this->matchModel.emplace("intersection_70","intersection_South_003b");
  this->matchModel.emplace("intersection_16","intersection_East_003b");
  this->matchModel.emplace("intersection_17","intersection_West_003b");
  this->matchModel.emplace("intersection_12","intersection_South_003c");
  this->matchModel.emplace("intersection_13","intersection_East_003c");
  this->matchModel.emplace("intersection_15","intersection_West_003c");


  this->matchModel.emplace("intersection_47","intersection_South_004a");
  this->matchModel.emplace("intersection_12_clone_0","intersection_West_004a");
  this->matchModel.emplace("intersection_72","intersection_South_004b");
  this->matchModel.emplace("intersection_7","intersection_North_004b");
  this->matchModel.emplace("intersection_14","intersection_West_004b");
  this->matchModel.emplace("intersection_55","intersection_South_004c");
  this->matchModel.emplace("intersection_21","intersection_North_004c");
  this->matchModel.emplace("intersection_19","intersection_West_004c");
  this->matchModel.emplace("intersection_73","intersection_South_004d");
  this->matchModel.emplace("intersection_54","intersection_North_004d");
  this->matchModel.emplace("intersection_64","intersection_West_004d");
  this->matchModel.emplace("intersection_52","intersection_South_004e");
  this->matchModel.emplace("intersection_53","intersection_North_004e");
  this->matchModel.emplace("intersection_61","intersection_West_004e");
  this->matchModel.emplace("intersection_74","intersection_South_004f");
  this->matchModel.emplace("intersection","intersection_North_004f");
  this->matchModel.emplace("intersection_26","intersection_West_004f");
  this->matchModel.emplace("intersection_43","intersection_South_004g");
  this->matchModel.emplace("intersection_44","intersection_North_004g");
  this->matchModel.emplace("intersection_27","intersection_West_004g");
  this->matchModel.emplace("intersection_36","intersection_South_004h");
  this->matchModel.emplace("intersection_39","intersection_North_004h");
  this->matchModel.emplace("intersection_10","intersection_West_004h");
  this->matchModel.emplace("intersection_35","intersection_South_004i");
  this->matchModel.emplace("intersection_36_clone","intersection_North_004i");
  this->matchModel.emplace("intersection_1","intersection_West_004i");
  
  //Corridors
  this->matchModel.emplace("corridor","corridor_001a");
  this->matchModel.emplace("corridor_0","corridor_001b");
  this->matchModel.emplace("corridor_1","corridor_001c");
  this->matchModel.emplace("corridor_2","corridor_001d");

  this->matchModel.emplace("corridor_22","corridor_002a");
  this->matchModel.emplace("corridor_19","corridor_002b");
  this->matchModel.emplace("corridor_18","corridor_002c");
  this->matchModel.emplace("corridor_13","corridor_002d");
  this->matchModel.emplace("corridor_11","corridor_002e");
  this->matchModel.emplace("corridor_10","corridor_002f");
  this->matchModel.emplace("corridor_6","corridor_002g");

  this->matchModel.emplace("corridor_4","corridor_003a");
  this->matchModel.emplace("corridor_3","corridor_003b");
  this->matchModel.emplace("corridor_5","corridor_003c");

  this->matchModel.emplace("corridor_9","corridor_004a");
  this->matchModel.emplace("corridor_8","corridor_004b");
  this->matchModel.emplace("corridor_7","corridor_004c");
  this->matchModel.emplace("corridor_14","corridor_004d");
  this->matchModel.emplace("corridor_15","corridor_004e");
  this->matchModel.emplace("corridor_16","corridor_004f");
  this->matchModel.emplace("corridor_17","corridor_004g");
  this->matchModel.emplace("corridor_20","corridor_004h");
  this->matchModel.emplace("corridor_21","corridor_004i");
}

string LogicalCameraPlugin::checkIfRoom(string modelName){
  unordered_map<string,string>::const_iterator map_find = this->matchModel.find(modelName);
  if ( map_find == this->matchModel.end() )
  {
	gzdbg << "This is not a model to identify room: " << modelName << "!\n";
	return modelName;
  } else {
	gzdbg << "Matched the model " << map_find->first << "for a room: " << map_find->second << "!\n";
	return map_find->second;
  }
}

void LogicalCameraPlugin::findLogicalCamera()
{
  sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();

  for (physics::LinkPtr link : this->model->GetLinks())
  {
    for (unsigned int i =0; i < link->GetSensorCount(); ++i)
    {
      sensors::SensorPtr sensor = sensorManager->GetSensor(link->GetSensorName(i));

      if (sensor->Type() == "logical_camera")
      {
        this->sensor = sensor;
	break;
      }
    }

    if (this->sensor)
    {
      this->cameraLink = link;
      break;

    }
  }
}

void LogicalCameraPlugin::onImage(ConstLogicalCameraImagePtr &_msg)
{

  gazebo::msgs::LogicalCameraImage imageMsg;
  math::Vector3 modelPosition;
  math::Quaternion modelOrientation;
  math::Pose modelPose;

  for (int i = 0;i < _msg->model_size();++i)
  {
    modelPosition = math::Vector3(msgs::ConvertIgn(_msg->model(i).pose().position()));
    modelOrientation = math::Quaternion(msgs::ConvertIgn(_msg->model(i).pose().orientation()));
    modelPose = math::Pose(modelPosition, modelOrientation);
    
    std::string modelName = _msg->model(i).name();
    std::string modelFrameId = this->modelFramePrefix + modelName + "harsha";
    if ( modelName != "ground_plane" && modelName != "se1f1" && modelName.find("doorOutside") ==string::npos && modelName.find("wall_static") ==string::npos) { 
      //if ( modelName != "se1f1" ) { 
        gzdbg << "Model detected: " << modelName << "with pose" << modelPose << "\n";
        //this->publishTF(modelPose, this->name,modelFrameId);
     

      logical_camera_plugin::logicalImage msg;
      string finModelName = checkIfRoom(modelName);
      msg.modelName = finModelName;
      msg.pose_pos_x = modelPose.pos.x;
      msg.pose_pos_y = modelPose.pos.y;
      msg.pose_pos_z = modelPose.pos.z;
      msg.pose_rot_x = modelPose.rot.x;
      msg.pose_rot_y = modelPose.rot.y;
      msg.pose_rot_z = modelPose.rot.z;
      msg.pose_rot_w = modelPose.rot.w;
      this->imagePub.publish(msg);
     }
  }

}

void LogicalCameraPlugin::publishTF(
  const math::Pose &pose, const std::string &parentFrame, const std::string &frame)
{
    ros::Time currentTime = ros::Time::now();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform transform(qt, vt);
    transformBroadcaster->sendTransform(tf::StampedTransform(transform, currentTime, parentFrame, frame));
}
