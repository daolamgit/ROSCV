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
 
class simplecanny
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
 ros::Publisher pub ;
  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_; //image subscriber 
  image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
  std_msgs::String msg;
public:
 simplecanny()
    : it_(nh_)
  {
 
     image_sub_ = it_.subscribe("/cameras/left_hand_camera/image", 1, &simplecanny::imageCb, this);
     image_pub_= it_.advertise("out",1);
 
 
  }
 
  ~simplecanny()
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

      IplImage* out1 = cvCreateImage( cvGetSize((CvArr*) img), IPL_DEPTH_8U, 3 );   //make sure to feed the image(img) data to the parameters necessary for canny edge output 
      IplImage* gray_out = cvCreateImage( cvGetSize( (CvArr*)img), IPL_DEPTH_8U, 1 ); 
      IplImage* canny_out = cvCreateImage( cvGetSize( (CvArr*) img), IPL_DEPTH_8U, 1 );
      IplImage* gray_out1=cvCreateImage( cvGetSize((CvArr*) img), IPL_DEPTH_8U, 3 );
     // IplImage* img1 = cvCreateImage( cvGetSize( (CvArr*)img), IPL_DEPTH_8U, 3 ); 
      cvCvtColor(img, gray_out, CV_BGR2GRAY);
      cvSmooth(gray_out, gray_out, CV_GAUSSIAN, 9, 9); 
      cvCanny( gray_out, canny_out, 50, 125, 3 );
      cvCvtColor(canny_out ,gray_out1, CV_GRAY2BGR);
      //cvShowImage( "Camera Feed",img);
      //imshow(" Camera Feed 1", img1);	
      //cvShowImage( " CANNY EDGE DETECTION ",gray_out1);
      cvWaitKey(2);   
/*

 	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols >60)
	  cv::circle( cv_ptr->image, cv::Point( 50, 50), 100, CV_RGB( 255, 0, 0));
	cv::imshow( WINDOW, cv_ptr ->image);
	cv:waitKey( 3);
*/	
	cv_ptr->image = gray_out1;
	image_pub_.publish( cv_ptr->toImageMsg());
}
};
 
 
 
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_canny");
  simplecanny ic;
  ros::spin();
 
  return 0;
}
