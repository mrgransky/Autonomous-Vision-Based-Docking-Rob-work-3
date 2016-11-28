#include "visualization.h"

using namespace std;
using namespace cv;


int Visualization::H_MIN = 27;
int Visualization::H_MAX = 98;
int Visualization::S_MIN = 89;
int Visualization::S_MAX = 131;
int Visualization::V_MIN = 77;
int Visualization::V_MAX = 181;
const int Visualization::FRAME_WIDTH = 640;
const int Visualization::FRAME_HEIGHT = 480;
const int Visualization::MAX_NUM_OBJECTS = 50;

const int Visualization::MIN_OBJECT_AREA = 20 * 20;
const int Visualization::MAX_OBJECT_AREA = Visualization::FRAME_HEIGHT*Visualization::FRAME_WIDTH / 1.5;


const string Visualization::windowName = "Original Image";
const string Visualization::windowName1 = "HSV Image";
const string Visualization::windowName2 = "Thresholded Image";
const string Visualization::windowName3 = "After Morphological Operations";

const string Visualization::trackbarWindowName = "Trackbars";
const string Visualization::btnWindowName = "GUI";

bool Visualization::trackObjects = true;
bool Visualization::useMorphOps = true;

int Visualization::x = 0;
int Visualization::y = 0;

int Visualization::savedX = 0;
int Visualization::savedY = 0;

Visualization::Visualization(QObject *parent) : QObject(parent)
{

}

void Visualization::on_trackbar(int, void*)
{//This function gets called whenever a
    // trackbar position is changed
}

string intToString(int number){

    std::stringstream ss;
    ss << number;
    return ss.str();
}

void Visualization::createTrackbars(){
    //create window for trackbars
    namedWindow(trackbarWindowName, 0);
    //create memory to store trackbar name on window
    char TrackbarName[50];
    sprintf(TrackbarName, "H_MIN", H_MIN);
    sprintf(TrackbarName, "H_MAX", H_MAX);
    sprintf(TrackbarName, "S_MIN", S_MIN);
    sprintf(TrackbarName, "S_MAX", S_MAX);
    sprintf(TrackbarName, "V_MIN", V_MIN);
    sprintf(TrackbarName, "V_MAX", V_MAX);


    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    cv::createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
    cv::createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
    cv::createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
    cv::createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
    cv::createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
    cv::createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);

}

void Visualization::drawObject(int x, int y, Mat &frame){

    //use some of the openCV drawing functions to draw crosshairs
    //on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

    circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
    if (y - 25>0)
        line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
    else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
    if (y + 25<FRAME_HEIGHT)
        line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
    else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
    if (x - 25>0)
        line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
    else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
    if (x + 25<FRAME_WIDTH)
        line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
    else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

    putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

}

void Visualization::morphOps(Mat &thresh){

    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);


    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
}

void Visualization::trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if (numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                    x = moment.m10 / area;
                    y = moment.m01 / area;
                    objectFound = true;
                    refArea = area;
                }
                else objectFound = false;
            }
            //let user know you found an object
            if (objectFound == true){
                putText(cameraFeed, "Tracking Object!", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                //draw object location on screen
                drawObject(x, y, cameraFeed);
                getPOS(x,y);
            }

        }
             else
                putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
                noObjFound();
    }
    return;
}

void Visualization::progStart()
{
    createTrackbars();
    //open video stream from either webcam or biult-in camera

    const std::string vsa = "http://192.168.0.101:8080/video?x.mjpeg";
    capture.open(vsa);
   // capture.open(1);          // ******* 0 =built-in, 1 = webcam ********

    //set height and width of capture frame
    capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    //start an infinite loop where webcam feed is copied to cameraFeed matrix
    //all of our operations will be performed within this loop
    while (1){
        //store image to matrix
        capture.read(cameraFeed);
        //convert frame from BGR to HSV colorspace
        cv::cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
        //filter HSV image between values and store filtered image to
        //threshold matrix
        cv::inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
        //perform morphological operations on thresholded image to eliminate noise
        //and emphasize the filtered object(s)
        if (useMorphOps)
            morphOps(threshold);
        //pass in thresholded frame to our object tracking function
        //this function will return the x and y coordinates of the
        //filtered object
        if (trackObjects)
            trackFilteredObject(x, y, threshold, cameraFeed);

        //show frames
        cv::imshow(windowName2, threshold);
        cv::imshow(windowName, cameraFeed);
        cv::imshow(windowName1, HSV);

        //delay 30ms so that screen can refresh.
        //image will not appear without this waitKey() command
        waitKey(30);

    }
}

void Visualization::getPOS(int inpX, int inpY)
{
    savedX = inpX;
    savedY = inpY;
     cout << "\n x = " << savedX;
     cout << "\n y = " << savedY;
}

void Visualization::noObjFound()
{
    cout << "\nOops! no object... \n\n";
}

int Visualization::getPosX()
{
    return x;
}

int Visualization::getPOSY()
{
    return y;
}
