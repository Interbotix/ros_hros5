/* object_handler_node.cpp
 * Announces newly detected object names as they come into view from ORK
 * I'm awesome
 */
#include <ros/ros.h>

#include <iostream>
#include <algorithm>
#include <vector>

#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <object_recognition_core/db/prototypes/object_info.h>
#include <object_recognition_ros/object_info_cache.h>

using namespace std;

//TODO: no object count for each item. to prevent false negatives with multiple items in view
static int no_object_count;
static std::vector< std::string > c_existing_detections;
static int announce_object_delay;

object_recognition_ros::ObjectInfoDiskCache info_cache_;

//http://docs.ros.org/hydro/api/object_recognition_msgs/html/msg/RecognizedObjectArray.html
void callback(const object_recognition_msgs::RecognizedObjectArray& msg)
{
  if ( msg.objects.size() == 0 )
  {
    announce_object_delay = 0;
    ROS_WARN( "No objects detected (%d)", no_object_count );
    if ( ++no_object_count > 2 )
    {
      no_object_count = 0;
      c_existing_detections.clear();
    }
    return;
  }

  no_object_count = 0;
  std::vector< std::string > c_detected_objects;

  for (size_t i_msg = 0; i_msg < msg.objects.size(); ++i_msg) 
  {
    if ( isnan( msg.objects[i_msg].confidence ) )
    {
      continue;
    }

    const object_recognition_msgs::RecognizedObject& object = msg.objects[i_msg];
   
    // Check if we already have loaded the object
    object_recognition_core::prototypes::ObjectInfo object_info;
    try 
    {
      info_cache_.getInfo(object.type, object_info);
    } 
    catch(...) 
    {
      ROS_WARN_STREAM("Object " << object.type.key << " not found in database.");
      return;
    }

    // Get the name of the object
    std::string name;
    if (object_info.has_field("name"))
      name = object_info.get_field<std::string>("name");

    ROS_INFO( "Object(%0.2f)%% Name: %s", msg.objects[i_msg].confidence, name.c_str() );


    if ( msg.objects[i_msg].confidence < 0.91 ) ROS_WARN( "Confidence too low to accept detection" );
    //else 
    c_detected_objects.push_back( name );
  }

  ROS_INFO( "Objects in view: %d", (int)c_detected_objects.size() );

  if ( announce_object_delay > 4 )
  {
    for ( size_t i=0; i < c_detected_objects.size(); ++ i )
    {
      std::vector< std::string >::iterator it;
      it = find( c_existing_detections.begin(), c_existing_detections.end(), c_detected_objects[i] );
      if ( it == c_existing_detections.end() )
      {
        ROS_INFO_STREAM( "NEW detection: " << c_detected_objects[i] );
        std::stringstream ss;
        ss << "espeak \"I see a " << c_detected_objects[i] << "\"";
        system( ss.str().c_str() );
      }
    }
    c_existing_detections = c_detected_objects;
  }
  else
  {
    ++announce_object_delay;
    ROS_INFO( "DELAY %d", announce_object_delay );
  }
  
} 

int main(int argc,char **argv)
{
  no_object_count = 0;
  announce_object_delay = 0;

  ros::init(argc, argv, "object_handler_node");
  
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/recognized_object_array", 1, &callback);
  
  ros::spin();
 
  return 0;
}
