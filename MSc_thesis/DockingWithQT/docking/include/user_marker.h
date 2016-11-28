#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "gotodocking.h"
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

        void make6DofMarker(unsigned int interaction_mode,const tf::Vector3& position);

        void MarkerStart();
        
        static double savedX;
        
        static double savedY;

        static double savedZ;
        
        static double posX;
        
        static double posY;

        static double posZ;
        
        
        static double savedOrienW;
        static double savedOrienX;
        static double savedOrienY;
        static double savedOrienZ;
        
        
        static double orienW;
        static double orienX;
        static double orienY;
        static double orienZ;

private:

        GoToDocking *robotmoving;
        
        Marker makeBox( InteractiveMarker &msg );

        InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg );

        MenuHandler marker_menu;
        
        ros::Timer frame_timer;

        ros::NodeHandle n;

        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server;

        void frameCallback(const ros::TimerEvent&);

        void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        
        void CoordinatePose(double inpX, double inpY, double inpZ);
        
        void CoordinateOrien(double w, double roll, double pitch, double yaw);
        
        void CreateMenu();
        
        void saveCoordinate();
        
        void setMarCoor(const tf::Transform& inpMarPos);
        
        void printdummy();
};
