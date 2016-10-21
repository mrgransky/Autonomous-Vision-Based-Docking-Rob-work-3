#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>

using namespace visualization_msgs;
using namespace interactive_markers;

class MarkerForDocking
{

public:

        // Constructor
        MarkerForDocking();

        // Destructor
        ~MarkerForDocking();


        static uint32_t counter;

        static double posX;
        
        static double posY;

        static double posZ;
        
        static double orienW;
        static double orienX;
        static double orienY;
        static double orienZ;

        void make6DofMarker(unsigned int interaction_mode,const tf::Vector3& position);
        
        void makeMenuMarker(const tf::Vector3& position);

        void MarkerStart();

private:

        Marker makeBox( InteractiveMarker &msg );

        InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg );

        ros::Timer frame_timer;
        
        MenuHandler marker_menu;

        ros::NodeHandle n;

        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server;

        void frameCallback(const ros::TimerEvent&);

        void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        
        void printdummy(const tf::Transform& inpMarPos);
        
        void addMenuTomarker();

};
