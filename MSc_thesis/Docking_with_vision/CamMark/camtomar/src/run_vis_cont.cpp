#include <iostream>
#include<VisionControl.h>

int main(int argc, char** argv)
{
      ros::init(argc, argv, "aruco_tf_publisher");
        
        PID4Docking controller;
        controller.ProgStart(argc,argv);

        return 0;
}
