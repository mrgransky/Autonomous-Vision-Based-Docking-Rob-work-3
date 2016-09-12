#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <unistd.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

using namespace cv;
using namespace aruco;
using namespace std;

class PID4Docking
{
private:

        ros::NodeHandle node_vis;
	ros::NodeHandle node_cont;	

	ros::Publisher commandPub;
       
        ros::Subscriber MarPose_Sub;
        ros::Publisher MarPose_pub;

        Mat img;
                
        string TheInputVideo;
        string TheIntrinsicFile;

        MarkerDetector MDetector;
        VideoCapture TheVideoCapturer;

        vector<Marker> TheMarkers;

        Mat TheInputImage,TheInputImageCopy;
        CameraParameters TheCameraParameters;
        
        tf::TransformBroadcaster broadcaster;
        
	tf::TransformListener ObjListener;
	
        // --- Camera broadcaster ---
        tf::TransformBroadcaster CAMbr;
        tf::Transform transformCAM;
        
        // --- Robot listener ---
        tf::TransformListener RobListener;
  
        static int Thresh1_min,Thresh2_min;
        
        static int Thresh1_max,Thresh2_max;
        
        bool keepMoving;
        
        static int ThresParam1,ThresParam2;
        
        static int ThePyrDownLevel;
        static float TheMarkerSize;
        
        static bool update_images,found,Go2RandomPose;
        
        bool readArguments ( int argc,char **argv );
        
        static const string trackbarWindowName;
        
        void Controller(double RefX, double MarPoseX, double refY, double MarPoseY, double refYAW, double MarPoseYAW, double dt);
        
        void dock(double VelX, double VelY, double omegaZ);

	void move2docking(double VelX_est, double VelY_est, double omegaZ_est);
	//double* Q; 
        
public:

  PID4Docking();
    
  ~PID4Docking();
  
  Mat getCurrentImage();

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  
  void camCB(const geometry_msgs::PoseStamped::ConstPtr& CamFB);
  
  static void cvTackBarEvents(int value,void* ptr);
  
  void ProgStart(int argc,char** argv);
  
  void createTrackbars();
  
  void myhandler(int value);

  void GenerateRandomVal();
  void Undocking(double X_rand, double Y_rand, double theta_rand);

        void ContStart();

         static double Kp_x,Ki_x,Kd_x;
	 
	static double Kp_theta,Ki_theta,Kd_theta;

	

	static double safety_margin_X;
	double x_rand_SM;
        
        // ---- CONTROLL PARAMETERS ------ //
        static double prev_errorX;
        static double int_errorX;
        static double curr_errorX;
        static double diffX;
        static double p_termX;
        static double d_termX;
        static double i_termX;
        static double control_signalX;
        
        static double prev_errorY;
        static double int_errorY;
        static double curr_errorY;
        static double diffY;
        static double p_termY;
        static double d_termY;
        static double i_termY;
        static double control_signalY;
        
        static double prev_errorYAW;
        static double int_errorYAW;
        static double curr_errorYAW;
        static double diffYAW;
        static double p_termYAW;
        static double d_termYAW;
        static double i_termYAW;
        static double control_signalYAW;
        // ---- CONTROLL PARAMETERS ------ //
        
	static double x_new,y_new,theta_new; 	// inp (y,theta)
	static const double y_up,y_dwn,theta_up,theta_dwn;
	static double Kp_y,Ki_y,Kd_y; 		// actions (Kp,Ki,Kd)
	
	static double docking_counter;
	static double dock_started,dock_finished,docking_duration;
	
	static double TT_S,TT_E;

        static double zeroMin,zeroMax;

	static double y_dock_thresh,x_dock_thresh,theta_dock_thresh;
	static double x_thresh_undock,y_thresh_undock,theta_thresh_undock;

	static double rand_X_SM,rand_A_esp,rand_Y_esp;
	static double speed_reducer_X,speed_reducer_Y,speed_reducer_theta;
	static float r_off,p_off,y_off;
	static float roll,yaw,pitch;
	static double theta_with_offset,x_robCen2cam,y_robCen2cam;
	
	static double x_robINmar_coor,y_robINmar_coor;


        double marpose[6];
        double camPose[6];
        static const double RefPose[4];
        
};
