/*
 * depthSubtractBackground.cpp
 *
 *  Created on: Aug 1, 2013
 *      Author: dale
 */
/*
 * gaussianSubtractBackground.h
 *
 *  Created on: Jul 31, 2013
 *      Author: dale
 */

#ifndef DEPTHSUBTRACTBACKGROUND_H_
#define DEPTHSUBTRACTBACKGROUND_H_

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <cv_bridge/CvBridge.h>

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

static const char WINDOW[] = "depthSubtractBackground Image window";

const double pi = 3.1416;

class depthSubtractBackground
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
  //Mat bkg; //for learn the background

  int thresholdValue;
  string camera;
  //camera Info

public:
 depthSubtractBackground( const char* _camera)
    : it_(nh_)
  {
	 //get the camera side
	 camera = _camera;

	namedWindow( WINDOW, CV_WINDOW_AUTOSIZE);
	createTrackbar("Depth threshold", WINDOW, &thresholdValue, 5000 );
	//subscribe image topic
    string imageTopic = getImageTopic( camera);
	 cout << imageTopic << endl;
	image_sub_ = it_.subscribe( imageTopic, 1, &depthSubtractBackground::imageCb, this);


	//subscribe tf topic
	//sub = n.subscribe("tf", 1, &advancedSubtractBackground::getTfInfo, this);

	string objectTopic;
	objectTopic = "roscv/"+getCameraFullName( camera)+ "/objectROI";
	image_pub_ = it_.advertise(objectTopic, 1);
  }


  ~depthSubtractBackground()
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

	  {
		  if (msg->encoding.find( "bayer") == std::string::npos)
		  {
			  //its not an image at all
		  	  img1 = cv_bridge::toCvShare( msg, msg->encoding)->image;
		  }
	  }
	}

	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("Unable to convert %s image to 32FC1", msg->encoding.c_str());

	  ROS_ERROR( "cv_bridge exception: %s", e.what());
	  return;
	}

	/////learn the background
	static int i=0;
	static Mat bkg = Mat::zeros( img1.size(), CV_32FC1);
	i++;
	if (i<10){ //the first 10 images are backround
		cout << i << endl;
		addWeighted( img1, .5, bkg, .5, 0.0, bkg);
		threshold( bkg, bkg, 3.0, 255, THRESH_TRUNC);
		return;
	}

	Mat thresholdOutput;
	threshold( img1, img1, 3.0, 255, THRESH_TRUNC);
	subtractImage = bkg - img1; //bkg > img1
	threshold( subtractImage, thresholdOutput, thresholdValue/1000.0, 255, THRESH_BINARY);


	double minVal, maxVal;
		minMaxLoc( img1, &minVal, &maxVal);
		Mat draw;
		cout << " MIn max " << minVal << " " << maxVal <<endl;
		img1.convertTo( draw, CV_8U, -255.0/(maxVal - minVal), 255-minVal*255.0/(maxVal - minVal));
		//if (maxVal > 0)
		//	img1 /= maxVal;
		imshow(WINDOW, draw);
		waitKey(100);




	//bg.getBackgroundImage( back);


	erode( thresholdOutput, thresholdOutput, Mat());
	dilate( thresholdOutput, thresholdOutput, Mat());

	imshow( "Threshold", thresholdOutput);
	//namedWindow( WINDOW, CV_WINDOW_AUTOSIZE);
	//convert to display

//	cout << subtractImage.size() <<endl;
//	imshow(WINDOW, subtractImage);
//
//	cvMoveWindow( WINDOW, 2*subtractImage.cols, 0);
	//imshow(WINDOW, img1);
	char ckey = waitKey(20);
	if (ckey == 'p') //pause program to look
		getchar();

	//publish image
	//cv_ptr->image = subtractImage;
	//image_pub_.publish(cv_ptr->toImageMsg());
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




