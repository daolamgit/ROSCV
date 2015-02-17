#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <cv_bridge/CvBridge.h>
#include <cvaux.h>
#include<math.h>
#include <cxcore.h>
//#include <highgui.h>
 
/*here is a simple program which demonstrates the use of ros and opencv to do image manipulations on video streams given out as image topics from the monocular vision
of robots,here the device used is a ardrone(quad-rotor).*/
 
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "Image window";
 
class captureBackground
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
 ros::Publisher pub ;
  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_; //image subscriber 
  image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
  std_msgs::String msg;
  string camera; //camera name

public:
 captureBackground(const char* _camera)
    : it_(nh_)
  {
	camera = _camera;

     //resolve the input /cameras/cam/image
//     string topic = nh_.resolveName( "image");
//     cout << topic << endl;
     cout << "press c for capture background \n";	
		 
     string imageTopic = "/cameras/"+ getCameraFullName( camera) + "/image";
     		cout << imageTopic << endl;

     image_sub_ = it_.subscribe( imageTopic, 1, &captureBackground::imageCb, this);
     image_pub_= it_.advertise("outLeft",1);
 
 
  }
 
  ~captureBackground()
  {
    cv::destroyWindow(WINDOW);
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
 
     //sensor_msgs::CvBridge bridge;//we need this object bridge for facilitating conversion from ros-img to opencv
	//IplImage* img = bridge.imgMsgToCv(msg,"bgr8");  //image being converted from ros to opencv using cvbridge
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy( msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR( "cv_bridge exception: %s", e.what());
	  return;
	}
	
	Mat img1 = cv_ptr->image;
	IplImage img2 = img1;
	IplImage* img = &img2;
/*

 	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols >60)
	  cv::circle( cv_ptr->image, cv::Point( 50, 50), 100, CV_RGB( 255, 0, 0));
	cv::imshow( WINDOW, cv_ptr ->image);
	cv:waitKey( 3);
*/	
	imshow( WINDOW, cv_ptr->image);
	char keyPress = waitKey(2);
	//cout << keyPress << endl;
	if (keyPress == 99) //capture background
	{
		//filename
		static int i = -1;
		i++;
		string filename;
		ostringstream order;
		order << i;
		filename = camera + order.str() +".jpg";
		cout << filename << " saved " <<endl;
		//save file name
		imwrite( filename.c_str(), cv_ptr->image);
	}
	//cout << keyPress << endl;
	//cv_ptr->image = gray_out1;
	//image_pub_.publish( cv_ptr->toImageMsg());
}

  string getCameraFullName( string camera)
  {
  	if (!strcmp(camera.c_str(),"left"))
  		return "left_hand_camera";
  	if (!strcmp(camera.c_str(),"right"))
  			return "right_hand_camera";
  	if (!strcmp(camera.c_str(),"head"))
  			return "head_camera";
  	if (!strcmp(camera.c_str(),"kinect"))
  			return "kinect_camera";
  }
};
 
 
 
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_capture");

  cout << "command: rosrun roscv subtractbackground left|right|head|kinect \n";

    if ((strcmp( argv[1], "left")) && (strcmp( argv[1], "right")) && (strcmp( argv[1], "head")) )
  	  exit(-1);

  if (ros::names::remap( "image") == "image") 
	{
		ROS_WARN( "command: \t$ ./captureBackground image:=<image topic> [transport]");

	}	
  captureBackground cB (argv[1]); //( (argc >1 ) ? argv[1] : "raw");
  ros::spin();
 
  return 0;
}
