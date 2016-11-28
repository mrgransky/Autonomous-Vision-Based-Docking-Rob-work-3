#include <iostream>
#include <fstream>
#include <sstream>
#include <array>
#include <math.h>
#include <unistd.h>
#include <cstdlib>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

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

#include<VisionControl.h>

using namespace cv;
using namespace aruco;
using namespace std;

float PID4Docking::TheMarkerSize = .1; // Default marker size

int PID4Docking::Thresh1_min = 75;
int PID4Docking::Thresh2_min = 10;

int PID4Docking::Thresh1_max = 300;
int PID4Docking::Thresh2_max = 300;

const string PID4Docking::trackbarWindowName = "Trackbars";

int PID4Docking::ThePyrDownLevel = 0;

bool PID4Docking::update_images = true;

bool PID4Docking::found = false;
bool PID4Docking::Go2RandomPose = false;

int PID4Docking::ThresParam1 = 0;
int PID4Docking::ThresParam2 = 0;

// ---- CONTROLL PARAMETERS ------ //
double PID4Docking::prev_errorX, PID4Docking::curr_errorX, PID4Docking::int_errorX, PID4Docking::diffX;

double PID4Docking::p_termX = 0;
double PID4Docking::i_termX = 0;
double PID4Docking::d_termX = 0;

double PID4Docking::prev_errorY, PID4Docking::curr_errorY, PID4Docking::int_errorY, PID4Docking::diffY;

double PID4Docking::p_termY = 0;
double PID4Docking::i_termY = 0;
double PID4Docking::d_termY = 0;

double PID4Docking::prev_errorYAW,PID4Docking::curr_errorYAW,PID4Docking::int_errorYAW,PID4Docking::diffYAW;

double PID4Docking::p_termYAW = 0;
double PID4Docking::i_termYAW = 0;
double PID4Docking::d_termYAW = 0;

double PID4Docking::control_signalX, PID4Docking::control_signalY, PID4Docking::control_signalYAW;
// ---- CONTROLL PARAMETERS ------ //

// ---- Ref. Values for Android Camera ---- //
//const double PID4Docking::RefPose[4] = {-.0957, 0.00638817 /* Y_ref*/ ,  0.308857 /* X_ref*/ , 0.17 /* theta_ref*/}; 

// ---- Ref. Values for Logitech Camera ---- //
const double PID4Docking::RefPose[4] = {-.0957, 0.0199 /* Y_ref*/ , 0.201 /* X_ref*/ , -0.0335 /* theta_ref*/}; 

// ----------------  PID gains---------------- //
double PID4Docking::Kp_y = .38; //.4
double PID4Docking::Ki_y = 0 ;// 0
double PID4Docking::Kd_y = 0.02; //.1

double PID4Docking::Kp_theta = .25;// .18
double PID4Docking::Ki_theta = 0; //* Ki_y; // .15 * Ki_y
double PID4Docking::Kd_theta = 0; //* Kd_y; // .0008
// ----------------  PID gains---------------- //


double PID4Docking::TT_S,PID4Docking::TT_E;
// random pose initialized
const double PID4Docking::y_up = .3; 
const double PID4Docking::y_dwn = -.1; 
const double PID4Docking::theta_dwn = -.7 /*-RefPose[3]*/; 
const double PID4Docking::theta_up = .7 /*RefPose[3]*/;

double PID4Docking::x_new,PID4Docking::y_new,PID4Docking::theta_new;
double PID4Docking::dock_started,PID4Docking::dock_finished,PID4Docking::docking_duration;

double PID4Docking::speed_reducer_X = 1;
double PID4Docking::speed_reducer_Y = 1;
double PID4Docking::speed_reducer_theta = 1;

// ------ offsets X, Y, theta for Docking ---------
double PID4Docking::x_dock_thresh = .001;
double PID4Docking::y_dock_thresh = .002; //.002
double PID4Docking::theta_dock_thresh = (CV_PI/180) * .5; // .5 deg.

double PID4Docking::safety_margin_X = .16; // safety margin X axis in docking process : 18 cm

// ------ offsets X, Y, theta for Undocking ---------
double PID4Docking::x_thresh_undock = .02;
double PID4Docking::y_thresh_undock = .015;
double PID4Docking::theta_thresh_undock = (CV_PI/180) * 3;

double PID4Docking::docking_counter = 1;

// ---- offsets for Roll, Pitch, Yaw ----//
float PID4Docking::p_off = CV_PI;
float PID4Docking::r_off = CV_PI/2;
float PID4Docking::y_off = CV_PI;

float PID4Docking::roll,PID4Docking::yaw,PID4Docking::pitch;
double PID4Docking::theta_with_offset;

double PID4Docking::x_robCen2cam = -.95/2; // x_cam from the center of robot
double PID4Docking::y_robCen2cam = 0; // y_cam from the center of robot

double PID4Docking::x_robINmar_coor,PID4Docking::y_robINmar_coor;

PID4Docking::PID4Docking()
{	
	keepMoving = true;    
    
	/* initialize random seed: */
  	srand (time(NULL));

	x_rand_SM = RefPose[2] + .55; // 55 cm spreading domain in the x-axis while moving towards the random pose

    	// Publish pose message and buffer up to 100 messages
    	MarPose_pub = node_vis.advertise<geometry_msgs::PoseStamped>("/marker_pose", 100);
    	commandPub = node_cont.advertise<geometry_msgs::Twist>("/base_controller/command",100);
    
    	MarPose_Sub = node_vis.subscribe("/marker_pose",100,&PID4Docking::camCB,this);
}

  PID4Docking::~PID4Docking()
  {
	
  }
  Mat PID4Docking::getCurrentImage() 
  {
        return img;
  }

void PID4Docking::cvTackBarEvents(int value,void* ptr)
{
    PID4Docking* ic = (PID4Docking*)(ptr);
    ic-> myhandler(value);
}

void PID4Docking::myhandler(int value)
{
        if (Thresh1_min<3) Thresh1_min=3;
    
    if (Thresh1_min%2!=1) Thresh1_min++;
    
    if (ThresParam2<1) ThresParam2=1;
    
    ThresParam1 = Thresh1_min;
    ThresParam2 = Thresh2_min;
    
    MDetector.setThresholdParams(ThresParam1,ThresParam2);

    // Recompute
    MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters);
    // TheInputImageCopy is the output image for TheInputImage
    TheInputImage.copyTo(TheInputImageCopy);

    for (unsigned int i=0;i<TheMarkers.size();i++) 
    {
        TheMarkers[i].draw(TheInputImageCopy,Scalar(205,0,0),1);
    }
	
    imshow("INPUT IMAGE",TheInputImageCopy);
    //imshow("THRESHOLD IMAGE",MDetector.getThresholdedImage());
}
void PID4Docking::createTrackbars()
{    
	namedWindow(trackbarWindowName, 0);
	createTrackbar("ThresParam 1", trackbarWindowName, &Thresh1_min, Thresh1_max, cvTackBarEvents, this);
	createTrackbar("ThresParam 2", trackbarWindowName, &Thresh2_min, Thresh2_max, cvTackBarEvents, this);
	
}
void PID4Docking::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    img = cv_ptr->image;
    
}

bool PID4Docking::readArguments ( int argc,char **argv )
{
    if (argc<2) {
        cerr<< "Invalid number of arguments!\n" <<endl;
        cerr<< "Usage: (in.avi|live|copy) [intrinsics.yml] [marker's width/height]" <<endl;
        return false;
    }
    TheInputVideo=argv[1];
    if (argc>=3)
        TheIntrinsicFile=argv[2];
    if (argc>=4)
        TheMarkerSize=atof(argv[3]);
    if (argc==3)
        cerr<< "NOTE: You need makersize to see 3d info!" <<endl;
    return true;
}

void PID4Docking::ProgStart(int argc,char** argv)
{
	// Show images, press "SPACE" to diable image
        // rendering to save CPU time
        
	if (readArguments(argc,argv)==false)
	{
		cerr<<"check the authenticity of your file again!"<<endl;
		node_vis.shutdown();
	}
	createTrackbars();
	
	// IP address for raw3_lund    
        const std::string vsa = "http://192.168.0.101:8080/video?x.mjpeg";
        // -- publishing video stream with Android Camera--
        
    TheVideoCapturer.open(vsa);

    //TheVideoCapturer.open(0);
	
	// Check video is open
	if (!TheVideoCapturer.isOpened())
	{
		cerr<<"Could not open video!!"<<endl;
		node_vis.shutdown();
	}
	dock_started = ros::Time::now().toSec();
	// Read first image to get the dimensions
	TheVideoCapturer>>TheInputImage;

	// Read camera parameters if passed
	if (TheIntrinsicFile!="") {
		TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
		TheCameraParameters.resize(TheInputImage.size());
	}

	// Configure other parameters
	if (ThePyrDownLevel>0)
	{
		MDetector.pyrDown(ThePyrDownLevel);
        }
        
	MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
	
	char key=0;
	int index=0;

	//ros::Rate rate(10);
	// Capture until press ESC or until the end of the video
	while ((key != 'x') && (key!=27) && TheVideoCapturer.grab() && node_vis.ok() && keepMoving)
	{
		TT_S = ros::Time::now().toSec();

	/* --- For the purpose of showing the time ---
	Mat frame;
        TheVideoCapturer >> frame; // get a new frame from camera
	imshow("video stream",frame);
	waitKey(30); // 30 ms */

		
if (TheVideoCapturer.retrieve(TheInputImage))
{

// Detection of markers in the image passed
		MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize);
		TheInputImage.copyTo(TheInputImageCopy);
		geometry_msgs::PoseStamped msg;
                
                float x_t, y_t, z_t;
			
		if (TheMarkers.size()>0)
		{
		        found = true;
		        //ROS_INFO("MARKER FOUND!!! ... \n");
		}else
		{
		        found = false;
			keepMoving = false;
			ROS_INFO_STREAM("Marker is lost, successful docking trials : " << (docking_counter - 1) << "\n");				
		        //RandomPose(x_new,y_new,theta_new);			
			//move2docking(-control_signalX, -control_signalY, control_signalYAW);
		}
			
		if (node_vis.ok() && found)
		{
		        //y_t = -TheMarkers[0].Tvec.at<Vec3f>(0,0)[0]; // changed !!!
			y_t = TheMarkers[0].Tvec.at<Vec3f>(0,0)[0];
			x_t = TheMarkers[0].Tvec.at<Vec3f>(0,0)[1];
			z_t = TheMarkers[0].Tvec.at<Vec3f>(0,0)[2];
		
			Mat R(3,3,cv::DataType<float>::type);

			// You need to apply cv::Rodrigues() in order to obatain angles wrt to camera coords
			Rodrigues(TheMarkers[0].Rvec,R);

			roll  = atan2(R.at<float>(1,0), R.at<float>(0,0));	
			pitch = atan2(-R.at<float>(2,0),pow((pow(R.at<float>(2,1),2)+pow(R.at<float>(2,2),2)),.5));
			yaw   = atan2(R.at<float>(2,1), R.at<float>(2,2)); // useful

                        // Adding Camera frame to Robot Frame ---
			tf::Quaternion quat_CAM = tf::createQuaternionFromRPY(0, 0, 0);
			broadcaster.sendTransform(tf::StampedTransform(tf::Transform(quat_CAM,tf::Vector3(.25, 0, .5)),ros::Time::now(),"/base_link","/camera"));
			
                        
                        // Adding Marker frame to Camera Frame ---
			tf::Quaternion quat_M = tf::createQuaternionFromRPY(roll,pitch,yaw);
			broadcaster.sendTransform(tf::StampedTransform(tf::Transform(quat_M,tf::Vector3(x_t, y_t, z_t)),ros::Time::now(),"/camera","/marker"));
			
				msg.header.frame_id = "/camera";
				
				// Publish Position
				msg.pose.position.x = x_t;
				msg.pose.position.y = y_t;
				msg.pose.position.z = z_t;
				
				// Publish Orientation
				msg.pose.orientation.x = roll;
				msg.pose.orientation.y = pitch;
				msg.pose.orientation.z = yaw;

				MarPose_pub.publish(msg);

			} 
			
			/*// Print other rectangles that contains no valid markers
			 for (unsigned int i=0;i<MDetector.getCandidates().size();i++) 
			{
				Marker m( MDetector.getCandidates()[i],10);
				m.draw(TheInputImageCopy,Scalar(0,255,0),2);
			}*/
					
			for (unsigned int i=0;i<TheMarkers.size();i++)
				{
					int currentMarID = TheMarkers[i].id;
					TheMarkers[i].draw(TheInputImageCopy,Scalar(0,255,0),2)	;
				
					CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
					CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

					//Marker ID to string
    					stringstream marker_id_string;
    					marker_id_string << "marker_ " << currentMarID;
				}

			if (update_images)
			{
				imshow("INPUT IMAGE",TheInputImageCopy);
				//imshow("THRESHOLD IMAGE",MDetector.getThresholdedImage());
			}
                
}else
{
        printf("retrieve failed\n");
}

key=cv::waitKey(30);
// If space is hit, don't render the image.

if (key == ' ')
{
	update_images = !update_images;
}

		ros::spinOnce();
		
		TT_E = ros::Time::now().toSec();
		ROS_INFO_STREAM(" visualization while loop duration = " << (TT_E - TT_S) <<" \n");

	}
}
void PID4Docking::camCB(const geometry_msgs::PoseStamped::ConstPtr& CamFB) // subscriber
{
camPose[0] = CamFB->pose.position.x; // not important!!!


camPose[1] = CamFB->pose.position.y; // y pose
camPose[2] = CamFB->pose.position.z; // x_rob

camPose[3] = CamFB->pose.orientation.x; //  Robot roll
camPose[4] = CamFB->pose.orientation.y; //  Robot pitch
camPose[5] = CamFB->pose.orientation.z; //  Robot yaw
        
	/*ROS_INFO_STREAM("--------- PID gains in trial no. " << docking_counter << " : ---------\n");
	ROS_INFO_STREAM(" Kp_y = " << Kp_y << " ,  Ki_y = " << Ki_y << " , Kd_y = " << Kd_y << "\n");        
	ROS_INFO_STREAM(" Kp_theta = " << Kp_theta << " ,  Ki_theta = " << Ki_theta << " , Kd_theta = " << Kd_theta << "\n");*/

	ROS_INFO_STREAM(" --------------------- Pose estimation ------------------ \n");
	
	// yaw has an offset of 180 deg.
	if (camPose[5]  > 0)
	{
		theta_with_offset = camPose[5] - y_off;
	} else
	{
		theta_with_offset = camPose[5] + y_off;
	}

	if (theta_with_offset > 0)
	{
		//x_robINmar_coor =  -camPose[2] * cos(theta_with_offset) + camPose[1] * sin(theta_with_offset);
		x_robINmar_coor =  camPose[2] * cos(theta_with_offset) - camPose[1] * sin(theta_with_offset);		
		y_robINmar_coor =  -camPose[2] * sin(theta_with_offset) + camPose[1] * cos(theta_with_offset);

	} else if (theta_with_offset < 0)
	{
		//x_robINmar_coor = -camPose[2] * cos(theta_with_offset) - camPose[1] * sin(theta_with_offset);
		x_robINmar_coor = camPose[2] * cos(theta_with_offset) - camPose[1] * sin(theta_with_offset);	
		y_robINmar_coor = -camPose[2] * sin(theta_with_offset) + camPose[1] * cos(theta_with_offset);
	} else
	{
		ROS_INFO_STREAM(" Mew condition should be added for theta! \n");
	}

	ROS_INFO_STREAM("theta = " << theta_with_offset << "rad. =~ " << theta_with_offset*(180.0/CV_PI) << " deg. \n");
	ROS_INFO_STREAM(" theta_ref = " << RefPose[3] << " rad. =~ " << (180/CV_PI) * RefPose[3] << " deg. \n");
	ROS_INFO_STREAM(" X_mar = " << x_robINmar_coor << " vs. X_ref = " << RefPose[2] << " \n");
        ROS_INFO_STREAM(" Y_mar = " << y_robINmar_coor << " vs. Y_ref = " << RefPose[1] << " \n");
	
        ROS_INFO_STREAM(" y_t = " << camPose[1] << " , x_t = " << camPose[2] << "\n");

	ROS_INFO_STREAM("------------------------------------------------------  \n ");

	if(Go2RandomPose == false)
	{
		ROS_INFO_STREAM("---------- MOVING TOWARDS DOCKING PLATFORM ---------  \n ");
		if (
            		(abs(RefPose[2] - x_robINmar_coor) <= x_dock_thresh)
            	)
        	{                        			
			dock_finished = ros::Time::now().toSec();
			docking_duration = dock_finished - dock_started;
			ROS_INFO_STREAM("docking No. " << docking_counter << " is finished in "<< docking_duration <<" sec, moving to new Random Pose\n");		
			keepMoving = false;
			GenerateRandomVal();
			docking_counter ++;
			speed_reducer_X = speed_reducer_Y = speed_reducer_theta = 1;
			//Go2RandomPose = true;

		// to make sure that y & theta are within the threshold...
        	} else if (abs(RefPose[2] - x_robINmar_coor) <= safety_margin_X)
		{
			if(
				(abs(RefPose[1] - y_robINmar_coor) > y_dock_thresh) || 
				(abs(RefPose[3] - theta_with_offset) > theta_dock_thresh)
			)
			{	
				ROS_INFO_STREAM(" delta_X < " << safety_margin_X << " m., Fixing Y or theta. \n ");     
				speed_reducer_Y = speed_reducer_theta = 1;		
				Controller(RefPose[2], RefPose[2], RefPose[1], y_robINmar_coor, RefPose[3], theta_with_offset,.1);
			} else if(
				(abs(RefPose[1] - y_robINmar_coor) <= y_dock_thresh) && 
				(abs(RefPose[3] - theta_with_offset) <= theta_dock_thresh)				
				)
			{
				ROS_INFO("y & theta fixed successfully, MOVING STRAIGHT AHEAD ... \n");

				/*speed_reducer_X = .06; // for x_dot = .1 =>.06*/
				speed_reducer_X = .04; // for x_dot = .15 =>.04  ---- optimal docking time and Y-axis offset ----
				/*speed_reducer_X = .0375; // for x_dot = .16 =>.0375*/

				Controller(RefPose[2], x_robINmar_coor, RefPose[1], RefPose[1], RefPose[3], RefPose[3],.1);
			}
		}else
        	{
			speed_reducer_X = 1;                	
			Controller(RefPose[2], x_robINmar_coor, RefPose[1], y_robINmar_coor, RefPose[3], theta_with_offset,.1);
		}
	} else
	{
  		ROS_INFO("---------- MOVING TOWARDS RANDOM POSE ---------\n");		
		Undocking(x_new,y_new,theta_new);
	}
}

void PID4Docking::Controller(double RefX, double MarPoseX, double RefY, double MarPoseY, double RefYAW, double MarPoseYAW, double dt)
{
	ROS_INFO_STREAM("--------------------- Controller started ----------------------\n "); 
        
	// -----------------X--------------------- //        
	if(abs(RefX - MarPoseX) > x_dock_thresh)
	{			
		/*// e(t) = setpoint - actual value;
        	curr_errorX = RefX - MarPoseX;
        
        	// Integrated error
        	int_errorX +=  curr_errorX * dt;
        	// differentiation
        	diffX = ((curr_errorX - prev_errorX) / dt);
        	// scalling
        	p_termX = Pos_Px * curr_errorX;
        	i_termX = Pos_Ix * int_errorX;
        	d_termX = Pos_Dx * diffX;
        	// control signal
        	control_signalX = p_termX + i_termX + d_termX;
        	prev_errorX = curr_errorX;*/
		
		//control_signalX = speed_reducer_X * 0.1;
		control_signalX = speed_reducer_X * 0.15;
		//control_signalX = speed_reducer_X * 0.16;        
        } else
	{
		control_signalX = 0;	// 5e-5
	}
        // -----------------Y--------------------- // 
	 
	if((RefY - MarPoseY) < -y_dock_thresh || (RefY - MarPoseY) > y_dock_thresh)
	{	
		// e(t) = setpoint - actual value;
        	curr_errorY = RefY - MarPoseY;

        	// Integrated error
        	int_errorY +=  curr_errorY * dt;
        	/*
        	// -- windup gaurd -- 
        	if (int_error < )
        	{}
       		else if ()
        	{}*/
        
        	// differentiation
        	diffY = ((curr_errorY - prev_errorY) / dt);
        
        	// scalling
        	p_termY = Kp_y * curr_errorY;
        	i_termY = Ki_y * int_errorY;
        	d_termY = Kd_y * diffY;

ROS_INFO_STREAM("pY = " << p_termY << ", iY = " << i_termY << " dY = " << d_termY<< " \n");	      
        	
        	control_signalY = p_termY + i_termY + d_termY;
		control_signalY = speed_reducer_Y * control_signalY;
        	prev_errorY = curr_errorY;
	
	} else if ((RefY - MarPoseY) <= y_dock_thresh && (RefY - MarPoseY) >= -y_dock_thresh)
	{
		control_signalY = 0;	
	}
        
	// -------------------YAW--------------------------//
       if(abs(RefYAW - MarPoseYAW) > theta_dock_thresh)
	{	
		//ROS_INFO_STREAM("REF = "<< RefYAW<< ", theta = "<< MarPoseYAW<< ".\n");
		// e(t) = setpoint - actual value;
		curr_errorYAW = RefYAW - MarPoseYAW;
        	// Integrated error
        	int_errorYAW +=  curr_errorYAW * dt;
        
        	// differentiation
        	diffYAW = ((curr_errorYAW - prev_errorYAW) / dt);

		//ROS_INFO_STREAM(" adjusting orientation! \n");	        		
		// scalling
        	p_termYAW = Kp_theta * curr_errorYAW;
       		i_termYAW = Ki_theta * int_errorYAW;
       		d_termYAW = Kd_theta * diffYAW;
        	
//ROS_INFO_STREAM("p_theta = " << p_termYAW << ", i_theta = " << i_termYAW << " d_theta = " << d_termYAW << " \n");	      
		// control signal
        	control_signalYAW = p_termYAW + i_termYAW + d_termYAW;
        	
		
        	// save the current error as the previous one
        	// for the next iteration.
        	prev_errorYAW = curr_errorYAW;
    
        } else if (abs(RefYAW - MarPoseYAW) <= theta_dock_thresh)
	{
		control_signalYAW = 0;	
	}

        /* ---   
	ROS_INFO_STREAM("Control signalX = " << control_signalX <<"\n");
	ROS_INFO_STREAM("Control signalY = " << control_signalY << "\n");
	ROS_INFO_STREAM("Control signalYAW = "<< control_signalYAW <<"\n");
	*/ 

	ROS_INFO_STREAM(" ---------------------- Controller ended ----------------------- \n");	
	dock(control_signalX, control_signalY, control_signalYAW);
}

void PID4Docking::dock(double VelX, double VelY, double omegaZ)
{
        ROS_INFO(".... REAL .... !");
        geometry_msgs::Twist msg;
        
	msg.linear.x = VelX;
	msg.linear.y = VelY;
	msg.angular.z = omegaZ;
	
	commandPub.publish(msg);
	
	ROS_INFO_STREAM(" Current speed of robot: \n" << msg << "");
}

void PID4Docking::move2docking(double VelX_est, double VelY_est, double omegaZ_est)
{
	
        ROS_INFO_STREAM(" Zmar = " << camPose[2] << " m. \n");
        ROS_INFO_STREAM(" Zref = " << RefPose[2] << " m. \n");
        
        ROS_INFO_STREAM(" Ymar = " << camPose[1] << " m. \n");
        ROS_INFO_STREAM(" Yref = " << RefPose[1] << " m. \n");
        
        ROS_INFO_STREAM(" rollmar = " << camPose[3] << " rad. \n");
        ROS_INFO_STREAM(" rollref = " << RefPose[3] << " rad. \n");
        
	ROS_INFO(".... ESTIMATION .... !\n");
        geometry_msgs::Twist msg;
        
        if (VelX_est == 0 && VelY_est == 0 && omegaZ_est == 0)
        {
                VelX_est = .0001;
                VelX_est = .0001;
                omegaZ_est = 0;
        }
        
	msg.linear.x = VelX_est;
	msg.linear.y = VelY_est;
	msg.angular.z = omegaZ_est;
	
	commandPub.publish(msg);
	
	ROS_INFO_STREAM(" Current ESTIMATED speed of robot: \n" << msg << ".\n");
}
// ---- Controller part -------- END ------

void PID4Docking::GenerateRandomVal()
{

	// ---------------- PID gains ------------------
	Kp_y = ((double) rand() / (RAND_MAX)) * (.76 - .4) + .4; 	//.1 < Kp < .76
	Ki_y = ((double) rand() / (RAND_MAX)) * .006;			// 0 < Ki < .006
	Kd_y = ((double) rand() / (RAND_MAX)) * .02;			// 0 < Kd < .01
	
	// ------------------ Generating Random Pose ------------------
	//x_new = ((double) rand() / (RAND_MAX)) * (1.1 - x_rand_SM) + x_rand_SM;
	x_new = 1.1;
	y_new = ((double) rand() / (RAND_MAX)) * (y_up - y_dwn) + y_dwn; // will be used for Q_Learning
	theta_new = ((double) rand() / (RAND_MAX)) * (theta_up - theta_dwn) + theta_dwn; // will be used for Q_Learning
}

void PID4Docking::Undocking(double X_rand, double Y_rand, double theta_rand)
{
	ROS_INFO_STREAM(" Xr = " << X_rand << ", Yr = " << Y_rand << ", Thetar = " << theta_rand << " rad ~ " << theta_rand * (180/CV_PI) << " deg\n");
	ROS_INFO_STREAM(" -------------------------------------------------------------- \n");

double vel_x,vel_y,omega_z;

geometry_msgs::Twist msg_new;
	// CCW ==>> w > 0 , CW ==>> w < 0
	// Leaving docking station, moving towards x-axis SM
	if (X_rand - camPose[2] > x_thresh_undock)
	{
		ROS_INFO_STREAM(" Adjusting X, moving backward ... \n");
		vel_x = -.04;
	} else if (X_rand - camPose[2] < -x_thresh_undock)
	{
		ROS_INFO_STREAM(" Adjusting X, moving forward ... \n");
		vel_x = .04;
	}else if (abs(X_rand - camPose[2]) <= x_thresh_undock)
	{
		ROS_INFO(" X-axis is fixed, adjusting Y & theta - axes ... \n");
		if ((camPose[1] - Y_rand > y_thresh_undock) && (theta_rand > 0))
		{
			if(abs(abs(camPose[3]) - abs(theta_rand)) > theta_thresh_undock)
			{
				ROS_INFO("moving 2 left side & CW rot. \n");			
				vel_y = .03;
				omega_z =  -.01;
			} else
			{
				ROS_INFO("CW rot. is fixed, only moving 2 left side ...\n");
				vel_y = .03;
			}	
		} else if ((camPose[1] - Y_rand < -y_thresh_undock) && (theta_rand < 0))
		{
			if(abs(abs(camPose[3]) - abs(theta_rand)) > theta_thresh_undock)
			{
				ROS_INFO("moving 2 right side & CCW rot. \n");			
				vel_y = -.03;
				omega_z = .01;
			}else
			{
				ROS_INFO("CCW rot. is fixed, only moving 2 right side ... \n");
				vel_y = -.03;
			}
		}else if (abs(camPose[1] - Y_rand) <= y_thresh_undock)
		{
			ROS_INFO(" Y-axis is fixed, adjusting theta-axis ... ! \n");			
			if (abs(abs(camPose[3]) - abs(theta_rand)) <= theta_thresh_undock)
			{			
				ROS_INFO(" Robot is in a new random Pose! \n");			
				//keepMoving = false;
				Go2RandomPose = false;
			} else
			{
				if(theta_rand > 0)
				{
					ROS_INFO_STREAM(" theta > 0 => Rob rot. is CW(-) \n");
					omega_z = -.01;
				} else
				{
					ROS_INFO_STREAM(" theta < 0 => Rob rot. is CCW(+) \n");
					omega_z = .01;			
				}
			}
		} else if ((camPose[1] - Y_rand > y_thresh_undock) && (theta_rand < 0))
		{ 
			if(abs(abs(camPose[3]) - abs(theta_rand)) > theta_thresh_undock)
			{
				ROS_INFO("moving 2 left side & CCW rot., chance of losing marker \n");			
				vel_y = .03;
				omega_z = .01;
			} else
			{
				ROS_INFO("CCW rot. is fixed, only moving 2 left side ... \n");
				vel_y = .03;
			}		
		} else if ((camPose[1] - Y_rand < -y_thresh_undock) && (theta_rand > 0))
		{
			if(abs(abs(camPose[3]) - abs(theta_rand)) > theta_thresh_undock)
			{
				ROS_INFO("moving 2 right side & CW rot., chance of losing marker \n");			
				vel_y = -.03;
				omega_z = -.01;
			} else
			{
				ROS_INFO("CW rot. is fixed, only moving 2 right side ... \n");
				vel_y = -.03;
			}
						
		} else
		{
			ROS_INFO(" New condition should be added! \n");			
			keepMoving = false;		
		}			
	}
	msg_new.linear.x = vel_x;
	msg_new.linear.y = vel_y;
	msg_new.angular.z = omega_z;

commandPub.publish(msg_new);
	
}

