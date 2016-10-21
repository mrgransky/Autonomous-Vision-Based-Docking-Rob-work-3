#include <ros/ros.h>
#include "user_marker.h"

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_6dof");

        MarkerForDocking marRun;
        marRun.MarkerStart();
        return 0; 

}
// %EndTag(main)%
