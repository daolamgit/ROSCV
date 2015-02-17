#include "depthSubtractBackground.h"


int main(int argc, char** argv)
{
	cout << "Subtracting background \n" ;
	cout << "command: rosrun roscv depthSubtractBackground kinectDepth \n";

  if ((strcmp( argv[1], "left")) && (strcmp( argv[1], "right")) && (strcmp( argv[1], "head")) && (strcmp( argv[1], "kinectDepth")) )
	  exit(-1);
  ros::init(argc, argv, "depth_subtract");
  if (ros::names::remap( "image") == "image")
  {
	  //ROS_WARN( "command: \t$ ./advancedSubtractBackground image:=<image topic>");
	  //exit(-1);(argc>1) ? argv[1]: "kinect"
  }
  depthSubtractBackground gB( argv[1]);
  ros::spin();
  
  return 0;
}
