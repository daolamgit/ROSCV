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
#include<math.h>
#include <cxcore.h>

#include "sensor_msgs/CameraInfo.h"
#include "tf/transform_listener.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <fstream>
//#include <highgui.h>
 
/*here is a simple program which demonstrates the use of ros and opencv to do image manipulations on video streams given out as image topics from the monocular vision
of robots,here the device used is a ardrone(quad-rotor).*/
 
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
RNG rng(12345); 
 
static const char WINDOW[] = "Image window";
const float maxImageWidth = 1280;

const double Zo = -.91 +0.71; //base to the ground
const double pi = 3.14156;

//subtract Image: make it a member because it is processed by several function
//Mat subtractImage;

class computeObjectPose
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

	
  int thresholdValue;
  int noObject; //number of detected object in the view
  Mat subtractImage, avBkgImage; //for imageProc

  //for object pose
  Mat centers; //for undistort image
  //Mat undistortCenters; // image space
  Mat undistortPointsNormalized;
  Mat worldPoints; //X and Y in world frame
  vector <RotatedRect> objectRect;

  string camera;
  //camera Info

  //for the robot param: camera and tf
  Mat D; //Distort vector 
  Mat K; // camera Matrix 3x3
  //tf info
  double R[3][3];
  double t[3];

  int currentImageWidth;

public:
 computeObjectPose( const char* _camera)
    : it_(nh_)
  {

	 //get the camera side
	 camera = _camera;

	thresholdValue = 100;	

	namedWindow( WINDOW, CV_WINDOW_AUTOSIZE);
  	createTrackbar( "Threshold:", WINDOW, &thresholdValue, 255 );
	
	avBkgImage = computeAvBkgImage( 5 ); //use 5 Bkg images


	//subscribe camera info topic
	string cameraInfoTopic = "/cameras/"+ getCameraFullName( camera) +"/camera_info";
	cout << cameraInfoTopic << endl;
	sub = n.subscribe(cameraInfoTopic, 1,	 &computeObjectPose::getCameraInfo, this);
	
	//subscribe image topic
	string imageTopic = "/cameras/"+ getCameraFullName( camera) + "/image";
		cout << imageTopic << endl;
	image_sub_ = it_.subscribe( imageTopic, 1, &computeObjectPose::imageCb, this);
	image_pub_= it_.advertise("outLeft",1);

	//subscribe tf topic
	//sub = n.subscribe("tf", 1, &computeObjectPose::getTfInfo, this);
  }
 

  ~computeObjectPose()
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
	//IplImage* img = &img2;
	
	//use opencv2 i.e. C++
	//subtract image
	//load an average gray bkg
	//sub the img1 from the avBkg
	//blob detect
	//output the blob properties: can be used for grasp
	
	////get the average bkground
	

	
	/////read the new image and subtract from the aveBkgImage
	
	Mat imgGray;
	cvtColor( img1, imgGray, CV_RGB2GRAY); 
	subtractImage = imgGray - avBkgImage;


	//namedWindow( WINDOW, CV_WINDOW_AUTOSIZE);
	imshow(WINDOW, subtractImage);
	cvMoveWindow( WINDOW, 2*subtractImage.cols, 0);
	//imshow(WINDOW, img1);
	char ckey = waitKey(20);
	if (ckey == 'p') //pause program to look
		getchar();
	//update the the dectected object windows
	//compute the bounding box, size, orientation
	thresh_callback(thresholdValue, 0);

	if (noObject>0)
	{
		//important
		undistortPointsNormalized = computeUndistortCenterandOrientation( centers);
		//however we use undistortCenterNormalized for computing
		cout << undistortPointsNormalized << endl;

		//ofstream myfile1 ("undistortCenters.txt");
		//myfile1 << undistortCenters ;
		//myfile1.close();

		getTfInfo();
		writeMatrix3(R, "R");
		writeMatrix1(t, 3 ,1, "t");

		//the error is so large. why?
		compute3Dcoordinate( worldPoints, undistortPointsNormalized, R, t);

		ofstream myfile2 ("worldPoints.txt");
		//myfile2 << worldPoints ;
		//myfile2.close();

		//advertise, publish the centers
		//using streamstring
		string objectTopic;
		objectTopic = "roscv/"+getCameraFullName( camera)+ "/objectPose";
		pub = n.advertise< std_msgs::String> (objectTopic, 1);
		std_msgs::String msg;
		msg.data = formatMsgToSend();
		cout << "Send: "<< msg.data << endl;
		pub.publish( msg);
		//getchar();
	}
	//exit(-1);
}

//i should organize the code after testing
//or i organize now for easy work in future
 void thresh_callback( int thresh, void* )  // don't even need this call back, i.e. can be right in ImgCallback

{
	Mat thresholdOutput;
	vector < vector <Point > > contours;
	vector < Vec4i> hierarchy;
	
	float smallestSize = 150;

	//reset the objectRect counter
	noObject = 0;
	objectRect.clear();

	//cout << "Callback Trackbar " << thresh << " \n";
	threshold( subtractImage, thresholdOutput, thresh, 255, THRESH_BINARY);
	namedWindow( "Blobs", CV_WINDOW_AUTOSIZE);
	//cvMoveWindow( "Blobs", subtractImage.cols, 0);
	imshow("Blobs", thresholdOutput);


	////detect the contours
	findContours (thresholdOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point( 0, 0));	
	vector <RotatedRect> minRect (contours.size());
	vector <Point2f> minEllipse (contours.size());	
	//cout << "Number of contours before filter: " << contours.size() << endl;
	
	//finding the bounding box and filter noise
	//////Consider morphology to connected components in case of 
	//glare object
	for (int i=0; i< contours.size(); i++)
	{	//aapproxPolyDP (Mat (contours[i]), contours_poly [i], 3, true);
		double area0 = contourArea( contours[i]);
		if (area0 > smallestSize)
		{
			minRect[noObject] = minAreaRect ( Mat( contours[i]) );

			//assign to the Classobject
			objectRect.push_back(minRect[noObject]);
			noObject ++;
		}
	}


	//cout << "Number of contours after filter: " << noObject << endl;

	//Drawing bounding box
	Mat drawing = Mat::zeros (thresholdOutput.size(), CV_8UC3);
	//thresholdOutput.copyTo( drawing);//Mat::zeros (thresholdOutput.size(), CV_8UC3);
	for (int i = 0; i< noObject; i++)
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform( 0, 255), rng.uniform(0, 255));
		//rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8,  0);
		Point2f rect_points[4];
		objectRect[i].points( rect_points);
		for (int j = 0; j< 4; j++)
			
			line( drawing, rect_points[j], rect_points[ (j+1)%4 ], color, 1, 8);

		cout << "Object "<< i <<" angle " << objectRect[i].angle << " center: " << objectRect[i].center << " size " << minRect[i].size <<endl ;

	}


	//convert to Mat to store all the centers
	//return: a matrix where each 2 column is a center and another point along the orientation
	//i.e c1 p1 c2 p2 c3 p3 ...
	// angle by RotatedRec is not a major so we have to compute
	//by finding 2 vertices on the longer side
	centers = Mat::zeros( 1, 3* noObject, CV_32FC2); //eacho obj 3 points
	for (int i = 0; i< noObject; i++)
	{
		//centers
		centers.at<Point2f>( 0 , 3*i) = minRect[i].center;

		//for Orientation Point
		Point2f rect_points[4];
		objectRect[i].points( rect_points);
		int mIndex; //1 or 2, 0 is always added

		if (l2Distance( rect_points[0], rect_points[1] ) > l2Distance( rect_points[0], rect_points[3] ))
			mIndex = 1;
		else
			mIndex = 3;

		//add first point
		centers.at<Point2f>(0, 3*i+1) = rect_points[0];
		centers.at<Point2f>(0, 3*i+2) = rect_points[mIndex];

		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform( 0, 255), rng.uniform(0, 255));
		circle( drawing, rect_points[0], 5, color);
		circle( drawing, rect_points[mIndex], 5, color);

	}

	namedWindow( "BoundingBox", CV_WINDOW_AUTOSIZE);
		//cvMoveWindow( "BoundingBox", 2*subtractImage.cols, 0);
	imshow("BoundingBox", drawing);
	if (noObject)
		cout << "Distort centers and Points: "<< centers <<endl;

	//for testing undistort

//	Mat undistortImage;
//	undistort( subtractImage, undistortImage, K, D);
//	namedWindow( "UndistortWindow", CV_WINDOW_AUTOSIZE);
//	imshow( "UndistortWindow", undistortImage);



	//undistortCenters = K* undistortCenters;
	 
}


Mat computeUndistortCenterandOrientation( Mat distortCenters)
{
	//return: a matrix where each 2 column is a center and another point along the orientation
	//i.e c1 p1 c2 p2 c3 p3 ...

	////	computeUndistortCenters(
	if (noObject >0){
		Mat scaledUndistortCenters;// = Mat::zeros( centers.rows, centers.cols, CV_64F);

		undistortPoints( distortCenters, scaledUndistortCenters, K, D); //this is a scaled coordinate

		//scale to image space
		undistortPointsNormalized = Mat::ones( 3, distortCenters.cols, CV_64F);
		for (int i = 0; i< distortCenters.cols; i++)
		{
			undistortPointsNormalized.at<double>(0, i) = scaledUndistortCenters.at<Point2f>(0, i).x;
			undistortPointsNormalized.at<double>(1, i) = scaledUndistortCenters.at<Point2f>(0, i).y;
		}
		//
		//cout<< undistortCenters <<endl;
		Mat undistortCenters;
		undistortCenters= K* undistortPointsNormalized;
		cout << "Undistort centers: " << undistortCenters <<endl;
		return undistortPointsNormalized;

	}
	


}

void getCameraInfo( const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	static int i= 0;
	i++;
	if (i==1)
	{
	//grab the cam param
		cout << "Camera INFO Callback \n";
		cout << "subscribe callback: \n" << *msg << endl;
		D = Mat::zeros( 5, 1, CV_64F);
		for (int i=0; i< 5; i++)
			D.at<double>(i) = msg->D[i];
		
		
		K = Mat::zeros( 3, 3, CV_64F);
		for (int i=0; i< 3; i++)
		  for (int j=0; j<3; j++)
			K.at<double>(i, j) = msg->K[i*3+ j];

		//adjust the optical center coordinates according to the size of the image
		//baxter camera doesn't change the resolution, it changes the size of the image so 
		//all the intrinsic params don't change except the optical, i.e
		//cNew = cMax* sizeNew/sizeMax;
		//Size currentImageSize avBkgImage.size();
		currentImageWidth = avBkgImage.cols;
		double scale = currentImageWidth/ maxImageWidth;
		K.at<double>(0, 2) = K.at<double>(0, 2)* scale;
		K.at<double>(1, 2) = K.at<double>(1, 2)* scale;	
	//get the info and unsubscribe
	
	//n.unsubscribe("/cameras/left_hand_camera/camera_info");
	//ROS_INFO("Left hand camera info :[%s]", msg->data.c_str());
	}
}


void getTfInfo ()
{
	tf::TransformListener tf;
	string frameFrom("base");
	string frameTo( "reference/"+getCameraFullName( camera));

	double r, p, y;

	tf::StampedTransform stampedTransform;
	tf.waitForTransform( frameFrom, frameTo, ros::Time(), ros::Duration(1.0));
	tf.lookupTransform( frameFrom, frameTo, ros::Time(), stampedTransform);

	tf::Matrix3x3 R3x3;
	R3x3 = stampedTransform.getBasis(); //getRPY( r, p, y);
	//Mat R = Mat::zeros(3, 3, CV_64F);

	//set R3x3 to R
	for (int i=0; i<3 ; i++)
	{
		tf::Vector3 v = R3x3.getRow(i);
		R[i][ 0] 	= v.getX();
		R[i][ 1]  	= v.getY();
		R[i][ 2]  	= v.getZ();
	}


			///R.at<double>(i,j) = R3x3.m_el[i][j];

		//remember this v is meter
	tf::Vector3 v = stampedTransform.getOrigin();
	//Mat t = Mat::zeros(3, 1, CV_64F);

	t[0] = v.getX();
	t[1] = v.getY();
	t[2] = v.getZ();
		//std::cout<< "Translation : " <<v.getX() <<", " <<v.getY() << ", " <<v.getZ()<< std::endl;
		//std::cout << r <<" "<<p <<" " << y <<std::endl;
	//cout << R <<endl;
	//cout << t <<endl;
	//writeMatrix3( R);
	//writeMatrix1(t, 3, 1);
}
 

void compute3Dcoordinate( Mat& worldPoints, const Mat imagePointsNormalized, const double R[][3], const double t[])
{
	imagePointsNormalized.copyTo( worldPoints);

	//cout << imagePoints.cols << endl;

	for (int i=0; i< imagePointsNormalized.cols; i++) //for each imagePoint
	{
		double x = imagePointsNormalized.at<double>(0, i);
		double y = imagePointsNormalized.at<double>(1, i);

		double Zc = (Zo - t[2])/(R[2][0]*x +R[2][1]*y +R[2][2]);
		double Xo = (R[0][0]*x +R[0][1]*y +R[0][2])* Zc + t[0];
		double Yo =  (R[1][0]*x +R[1][1]*y +R[1][2])* Zc + t[1];

		//check with camera model !
		worldPoints.at<double>( 0, i) = Xo;
		worldPoints.at<double>( 1, i) = Yo;
		worldPoints.at<double>( 2, i) = Zo;
	}

	cout << "World Points : \n" << worldPoints <<endl;
	//exit(-1);
	//getchar();
}

Mat computeAvBkgImage( int nBkgImages ) //must be in the same BkgImages
  {

	Mat bkgImage, avBkgImage;
	for (int i= 0; i<nBkgImages; i++)
	{
		string filename;
		ostringstream order;
		order << i;
		filename = camera + order.str() + ".jpg";
		bkgImage = imread( filename, CV_LOAD_IMAGE_GRAYSCALE);
		if (!bkgImage.data)

			cout << "Error loading file \n";
		if (i==0)
			avBkgImage = bkgImage;
		else
		//avBkgImage += bkgImage;
		addWeighted( avBkgImage, .5, bkgImage, .5, 0.0, avBkgImage);
	}

	return avBkgImage;
  }

template< class T> void writeMatrix1( T* matrix, const int row, const int col, const char* filename)
{
	ofstream file(filename);

	for (int i=0; i< row; i++)
	{
		for (int j=0; j<col; j++)
		{
			//T* p = matrix;
			cout << *(matrix) <<" ";
			file << *matrix <<" ";
			matrix++;
		}
		cout << endl;
		file << endl;
	}

	file.close();
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
	if (!strcmp(camera.c_str(),"kinect"))
		return "kinect_camera";
}

string formatMsgToSend( )
{// format the message to send : each line is X Y Z O W H
	stringstream ss;
	for (int i = 0; i< noObject; i++)
	{
		//get point pair
		float x0, y0, x1, y1, x2, y2, angle;
		x0 = worldPoints.at<double> (0, 3*i);
		y0 = worldPoints.at<double> (1, 3*i);
		x1 = worldPoints.at<double> (0, 3*i+ 1);
		y1 = worldPoints.at<double> (1, 3*i+ 1);
		x2 = worldPoints.at<double> (0, 3*i+ 2);
		y2 = worldPoints.at<double> (1, 3*i+ 2);

		ss << x0<< " ";
		ss << y0<< " ";
		ss << Zo<< " ";

		//ss << objectRect[i].angle << " ";
		//compute the object orientation in the world
		angle = atan2f(y2- y1, x2 - x1);
		angle *= 180 /pi;
		ss <<  angle<< " ";

		Size2f s =objectRect[i].size;
		ss << s.width << " ";
		ss << s.height << " ";
		ss << endl;
	}

	return ss.str();
}

float l2Distance(Point2f& p, Point2f& q) {
    Point2f diff = p - q;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

};



int main(int argc, char** argv)
{
	cout << "Subtracting background \n" ;
	cout << "command: rosrun roscv computeObjectPose left|right|head|kinect \n";

  if ((strcmp( argv[1], "left")) && (strcmp( argv[1], "right")) && (strcmp( argv[1], "head")) )
	  exit(-1);
  ros::init(argc, argv, "simple_subtract");
  if (ros::names::remap( "image") == "image")
  {
	  ROS_WARN( "command: \t$ ./computeObjectPose image:=<image topic>");
	  //exit(-1);(argc>1) ? argv[1]: "kinect"
  }
  computeObjectPose sB( argv[1]);
  ros::spin();
  
  return 0;
}
