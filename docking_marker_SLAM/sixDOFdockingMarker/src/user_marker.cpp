#include <ros/ros.h>
#include "user_marker.h"
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>



uint32_t MarkerForDocking::counter = 0;


double MarkerForDocking::posX = 0;
double MarkerForDocking::posY = 0;
double MarkerForDocking::posZ = 0;
double MarkerForDocking::orienW = 0;
double MarkerForDocking::orienX = 0;
double MarkerForDocking::orienY = 0;
double MarkerForDocking::orienZ = 0;


// constructor declaration
MarkerForDocking::MarkerForDocking()
{
        frame_timer = n.createTimer(ros::Duration(0.03), &MarkerForDocking::frameCallback,this);
        
        im_server=boost::shared_ptr<interactive_markers::InteractiveMarkerServer>(new 
        interactive_markers::InteractiveMarkerServer("simple_6dof", "user_marker"));
        
}

// destructor declaration
MarkerForDocking::~MarkerForDocking()
{
        
        im_server.reset();

}

// %Tag(Box)%
Marker MarkerForDocking::makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& MarkerForDocking::makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%


// %Tag(frameCallback)%
void MarkerForDocking::frameCallback(const ros::TimerEvent&)
{
  
static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

  counter++;
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void MarkerForDocking::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "\nFeedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << "\nat " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << "\nin frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    /*case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM("button is clicked!");
      break;
      */
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        {
                tf::Transform docking_pos;                      // a container
                tf::poseMsgToTF(feedback->pose,docking_pos);    // the container with coordinate
                printdummy(docking_pos);
                
                break;
        }
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    { 
        ROS_INFO_STREAM( s.str() << "\nNew Marker Coordinate :"
          << "\nPosition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\nOrientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << "\nat time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;
     }
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      {
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;
      }
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      {
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
      }
  }
  
  //im_server->applyChanges();
}
// %EndTag(processFeedback)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(Menu)%
void MarkerForDocking::makeMenuMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "Menu_Button";
  int_marker.description = "Menu_Button\n(Right Click)";
  
  InteractiveMarkerControl control;
  
  // insert a box
  Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  
  
  control.name = "menu_control";
  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.always_visible = true;
  int_marker.controls.push_back(control);
 
  
  im_server->insert(int_marker);
  im_server->setCallback(int_marker.name, boost::bind(&MarkerForDocking::processFeedback,this,_1));
  im_server->applyChanges();
  
  addMenuTomarker();
}
// %EndTag(Menu)%

void MarkerForDocking::addMenuTomarker()
{
        marker_menu.insert("Go to docking station",boost::bind(&MarkerForDocking::processFeedback,this,_1));
        marker_menu.apply( *im_server, "Menu_Button");
        im_server->applyChanges();
}


// %Tag(6DOF)%
void MarkerForDocking::make6DofMarker(unsigned int interaction_mode,const tf::Vector3& position)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "simple_6dof";
  int_marker.description = "6-DOF Marker Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;
    
    // ---------------- Move & Rotate in X direction -----------------
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // ---------------- Move & Rotate in Z direction -----------------
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // ---------------- Move & Rotate in Y direction -----------------
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);


  im_server->insert(int_marker);
  im_server->applyChanges();
  im_server->setCallback(int_marker.name, boost::bind( &MarkerForDocking::processFeedback, this, _1));
  
}
// %EndTag(6DOF)%

void MarkerForDocking::MarkerStart()
{
      
   ROS_INFO("Markers started... \n");     
  // simple 6 DOF ...      
  
  
  tf::Vector3 position;

  position = tf::Vector3( 0, 0, 0);
  make6DofMarker(visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position);
  
  position = tf::Vector3( 3,4, 0);
  makeMenuMarker(position);
  
  ros::spin();
  ros::Duration(0.1).sleep();

}

void MarkerForDocking::printdummy(const tf::Transform& inpMarPos)
{
        ROS_INFO("\nPosition is set, waiting for robot to move ... \n");
        InteractiveMarker int_marker;
        tf::poseTFToMsg(inpMarPos,int_marker.pose);
        
        
        posX = int_marker.pose.position.x;
        posY = int_marker.pose.position.y;
        posZ = int_marker.pose.position.z;
        
        orienW = int_marker.pose.orientation.w;
        orienX = int_marker.pose.orientation.x;
        orienY = int_marker.pose.orientation.y;
        orienZ = int_marker.pose.orientation.z;
                
}

