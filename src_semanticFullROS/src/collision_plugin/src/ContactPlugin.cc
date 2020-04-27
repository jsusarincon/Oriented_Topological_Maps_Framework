#include "collision_plugin/ContactPlugin.hh"
#include "collision_plugin/collision.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
   
  collisionPub = rosnode.advertise< collision_plugin::collision >("/collision", 1000);
  gzdbg << "Publishing to ROS topic: " << collisionPub.getTopic() << "\n";

 
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
	
  

  for (unsigned int i = 0; i < contacts.contact_size(); ++i){
    if (contacts.contact(i).collision2()!="ground_plane::link::collision"){

      //std::cout << "Collision between[" << contacts.contact(i).collision1()
      //<< "] and [" << contacts.contact(i).collision2() << "]\n";

      for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
      {
      /*  std::cout << j << "  Position:"
        << contacts.contact(i).position(j).x() << " "
        << contacts.contact(i).position(j).y() << " "
        << contacts.contact(i).position(j).z() << "\n";
        std::cout << "   Normal:"
        << contacts.contact(i).normal(j).x() << " "
        << contacts.contact(i).normal(j).y() << " "
        << contacts.contact(i).normal(j).z() << "\n";
        std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
      */


      collision_plugin::collision msg;
      msg.modelName = contacts.contact(i).collision1();
      msg.modelNameTouched = contacts.contact(i).collision2();
      msg.pos_x = contacts.contact(i).position(j).x();
      msg.pos_y = contacts.contact(i).position(j).y();
      msg.pos_z = contacts.contact(i).position(j).z();
      msg.norm_x = contacts.contact(i).normal(j).x();
      msg.norm_y = contacts.contact(i).normal(j).y();
      msg.norm_z = contacts.contact(i).normal(j).z();
      msg.depth = contacts.contact(i).depth(j);
      this->collisionPub.publish(msg);

      }
    }
  }
}
