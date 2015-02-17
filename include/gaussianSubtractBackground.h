/*
 * gaussianSubtractBackground.h
 *
 *  Created on: Jul 31, 2013
 *      Author: dale
 */

#ifndef GAUSSIANSUBTRACTBACKGROUND_H_
#define GAUSSIANSUBTRACTBACKGROUND_H_

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
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
#include <math.h>
#include <cxcore.h>

#include <fstream>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
RNG rng(12345);

static const char WINDOW[] = "gaussianSubtractBackground Image window";

const double pi = 3.1416;

class gaussianSubtractBackground
{
  ros::NodeHandle nh_;

  ros::NodeHandle n;
 ros::Publisher pub ; //for publish data, various topics
 ros::Subscriber sub; //for subscribe data

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_; //image subscriber
  image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)

  //maybe we dont even need it
  std_msgs::String msg;

  Mat subtractImage; //for imageProc
  BackgroundSubtractorMOG bg;//(10, 16, false);

    string camera;
  //camera Info

public:
 gaussianSubtractBackground( const char* _camera)
    : it_(nh_)
  {
	 //get the camera side
	 camera = _camera;

	namedWindow( WINDOW, CV_WINDOW_AUTOSIZE);

	//subscribe image topic
    string imageTopic = getImageTopic( camera);
	 cout << imageTopic << endl;
	image_sub_ = it_.subscribe( imageTopic, 1, &gaussianSubtractBackground::imageCb, this);


	//subscribe tf topic
	//sub = n.subscribe("tf", 1, &advancedSubtractBackground::getTfInfo, this);

	string objectTopic;
	objectTopic = "roscv/"+getCameraFullName( camera)+ "/objectROI";
	image_pub_ = it_.advertise(objectTopic, 1);
  }


  ~gaussianSubtractBackground()
  {
    cv::destroyWindow(WINDOW);
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

     //sensor_msgs::CvBridge bridge;//we need this object bridge for facilitating conversion from ros-img to opencv
	//IplImage* img = bridge.imgMsgToCv(msg,"bgr8");  //image being converted from ros to opencv using cvbridge
	cv_bridge::CvImagePtr cv_ptr;
	Mat img1;

	try
	{
	  if (camera=="kinectDepth")
		  // this does not work: cv_ptr = cv_bridge::toCvCopy( msg, enc::TYPE_32FC1);
	  {
		  if (msg->encoding.find( "bayer") == std::string::npos)
		  {
			  cout << "not work with depth image \n";
			  exit(-1);
		  	  img1 = cv_bridge::toCvShare( msg, msg->encoding)->image;
		  }
	  }

	  else
		  cv_ptr = cv_bridge::toCvCopy( msg, enc::BGR8);
	  	  img1 = cv_ptr->image;
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("Unable to convert %s image to 32FC1", msg->encoding.c_str());

	  ROS_ERROR( "cv_bridge exception: %s", e.what());
	  return;
	}


//only work with bayer
	bg.operator()( img1, subtractImage);
	//bg.getBackgroundImage( back);

	dilate( subtractImage, subtractImage, Mat());
	erode( subtractImage, subtractImage, Mat());


//	double minVal, maxVal;
//	minMaxLoc( subtractImage, &minVal, &maxVal);
//	cout << "Min max " << minVal << " "<< maxVal << endl;

	//namedWindow( WINDOW, CV_WINDOW_AUTOSIZE);
	imshow(WINDOW, subtractImage);
	cvMoveWindow( WINDOW, 2*subtractImage.cols, 0);
	//imshow(WINDOW, img1);
	char ckey = waitKey(20);
	if (ckey == 'p') //pause program to look
		getchar();

	//publish image
	cvtColor( subtractImage, subtractImage, CV_GRAY2RGB);
	cv_ptr->image = subtractImage;
	image_pub_.publish(cv_ptr->toImageMsg());
}



template< class T> void writeMatrix3( T matrix[3][3], const char* filename)
{
	ofstream file(filename);

	for (int i=0; i< 3; i++)
	{
		for (int j=0; j<3; j++)
		{
			//T* p = matrix;
			cout << matrix[i][j] <<" ";
			file << matrix[i][j] <<" ";
		}
		cout << endl;
		file << endl;
	}

	file.close();
}

string getCameraFullName( string camera)
{
	if (!strcmp(camera.c_str(),"left"))
		return "left_hand_camera";
	if (!strcmp(camera.c_str(),"right"))
		return "right_hand_camera";
	if (!strcmp(camera.c_str(),"head"))
		return "head_camera";
	if (!strcmp(camera.c_str(),"kinectRGB"))
		return "kinect_rgb";
	if (!strcmp(camera.c_str(),"kinectDepth"))
			return "kinect_depth";
}

string getImageTopic( string camera){
	if (!strcmp(camera.c_str(),"kinectRGB"))
		//return "camera/rgb/image_color";
		return "/camera/rgb/image_color";
	if (!strcmp(camera.c_str(),"kinectDepth"))
		//return "camera/rgb/image_color";
		return "/camera/depth_registered/image_rect";

	return "/cameras/"+ getCameraFullName( camera) + "/image";

  }

float l2Distance(Point2f& p, Point2f& q) {
    Point2f diff = p - q;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

};



#endif /* GAUSSIANSUBTRACTBACKGROUND_H_ */
