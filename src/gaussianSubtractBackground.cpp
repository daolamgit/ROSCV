#include "gaussianSubtractBackground.h"


int main(int argc, char** argv)
{
	cout << "Subtracting background \n" ;
	cout << "command: rosrun roscv gaussianSubtractBackground left|right|head|kinectRGB \n";

  if ((strcmp( argv[1], "left")) && (strcmp( argv[1], "right")) && (strcmp( argv[1], "head")) && (strcmp( argv[1], "kinectRGB")) )
	  exit(-1);
  ros::init(argc, argv, "gaussian_subtract");
  if (ros::names::remap( "image") == "image")
  {
	  //ROS_WARN( "command: \t$ ./advancedSubtractBackground image:=<image topic>");
	  //exit(-1);(argc>1) ? argv[1]: "kinect"
  }
  gaussianSubtractBackground gB( argv[1]);
  ros::spin();
  
  return 0;
}
