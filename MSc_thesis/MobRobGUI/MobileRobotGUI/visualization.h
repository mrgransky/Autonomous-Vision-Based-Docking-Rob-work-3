#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>

//#include <QMainWindow>
#include<QObject>

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

class Visualization: public QObject
{
    Q_OBJECT
public:
    explicit Visualization(QObject *parent = 0);

    static void on_trackbar(int, void*);

    static void createTrackbars();

    void drawObject(int x, int y, Mat &frame);

    void morphOps(Mat &thresh);

    void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed);

    void noObjFound();

    static int savedX;
    static int savedY;

    int getPosX();
    int getPOSY();

public slots:
    void progStart();

    void getPOS(int inpX, int inpY);




private:

    static bool trackObjects;
    static bool useMorphOps;

    //Matrix to store each frame of the webcam feed
    cv::Mat cameraFeed;

    //matrix storage for HSV image
    cv::Mat HSV;
    //matrix storage for binary threshold image
    cv::Mat threshold;
    //x and y values for the location of the object


    //video capture object to acquire webcam feed
    cv::VideoCapture capture;
    //initial min and max HSV filter values.
    //these will be changed using trackbars

    static int H_MIN;
    static int H_MAX;
    static int S_MIN;
    static int S_MAX;
    static int V_MIN;
    static int V_MAX;
    //default capture width and height
    static const int FRAME_WIDTH;
    static const int FRAME_HEIGHT;
    //max number of objects to be detected in frame
    static const int MAX_NUM_OBJECTS;
    //minimum and maximum object area
    static const int MIN_OBJECT_AREA;
    static const int MAX_OBJECT_AREA;

    //names that will appear at the top of each window
    static const string windowName;
    static const string windowName1;
    static const string windowName2;
    static const string windowName3;

    static const string trackbarWindowName;
    static const string btnWindowName;

    static int x, y;


};

#endif // VISUALIZATION_H
