#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/common/eigen.h>
//#include <pcl/common/impl/eigen.hpp>
#include <pcl/common/transforms.h>
//#include <Eigen/Core>
//#include <Eigen/Geo
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Geometry>

//#include <tf>
#include "tf/transform_listener.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

//For plane segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

//for PCA
#include <pcl/common/pca.h>

//for opencv
#include <opencv2/highgui/highgui.hpp>
//#include <cvaux.h>
//#include <cxcore.h>

//for message
#include "roscv/ObjectPosition.h"

using namespace std;
using namespace cv;

typedef struct
{
	float X;
	float Y;
	float Z;
	float ax; //x
	float ay; //y
	float az; //z
	float L;
	float W;
	float H;
} Object;

void transformfilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void cluster( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloudClusters);
void transform( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromCloud,
			 pcl::PointCloud<pcl::PointXYZRGB>::Ptr toCloud);
void computePose( const vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters, vector<Object>& obj);
void assign(roscv::ObjectPosition& msg, const Object& obj, const int id);


ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pub2;

int tiltAngle=24;
int sZ = 60, sY=22, sX=29; //position of slider
int controlMaxZ = 100, controlMinZ = 38;
int controlMaxX = 37, controlMinX = 30;
int controlMaxY = 56, controlMinY = 0;

float minZ = -.3, maxZ = .5;
float minX = 0.1, maxX = 1.5;
float minY = 0.1, maxY = 1.5;

float cZ = -.21;
float cX = .75;
float cY = .75;

const float pi = 3.1416;

void control()
{

	char c =waitKey(20);
	if (c=='p')
		getchar();
	//cout << maxZ << " " << maxX <<endl;
}

void cloud_cb( const sensor_msgs::PointCloud2ConstPtr& input)
{
	//pcl::PointCloud< pcl::PointXYZRGB> cloud, cloud_filtered;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new  pcl::PointCloud<pcl::PointXYZRGB>);
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters ;//(new  pcl::PointCloud<pcl::PointXYZRGB>);

	vector <Object> obj;
	control();

	pcl::fromROSMsg(*input, *cloud);

	//filter out
	transformfilter( cloud, cloud_filtered);

	//transform( cloud_filtered, cloud_filtered);

	//cluster (cloud_filtered, cloud_clustered );
	cluster (cloud_filtered, cloudClusters );


	computePose(cloudClusters , obj);
	//sensor_msgs::PointCloud2 msg1;
	for (int i=0; i< obj.size(); i++)
	{
		roscv::ObjectPosition msg;
		assign( msg, obj[i], i);

		//pcl::toROSMsg(*cloudClusters[0], msg);
		pub2.publish (msg);
	}


	sensor_msgs::PointCloud2 msg;
	pcl::toROSMsg(*cloud_filtered, msg);
	pub.publish (msg);


	//sensor_msgs::PointCloud2 msg1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDisplay (new  pcl::PointCloud<pcl::PointXYZRGB>);
	cloudDisplay->header = cloud_filtered->header;
	for (int i=0; i< cloudClusters.size(); i++)
	{
		*cloudDisplay += *cloudClusters[i];
		pcl::toROSMsg(*cloudDisplay, msg);
		pub1.publish (msg);
	}
	//cloud = input->data;
	//std::cout << "Cloud callback :" << input->width << " "<<" height :" << input->height <<std::endl;
}

void assign(roscv::ObjectPosition& msg, const Object& obj, const int id)
{
	msg.id = id;
	msg.position.x =  obj.X;
	msg.position.y =  obj.Y;
	msg.position.z =  obj.Z;

	float angle = atan2f( obj.ay, obj.ax);
	angle = angle*180/pi;
	msg.theta = angle;
//	msg.orientation.x = obj.ax;
//	msg.orientation.y = obj.ay;
//	msg.orientation.z = obj.az;

	msg.size.length = .5;
	msg.size.width = .01;
//	msg.size.x = obj.L;
//	msg.size.y = obj.W;
//	msg.size.z = obj.H;

}

void computePose( const vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters, vector<Object>& obj)
{
	pcl::PCA< pcl::PointXYZRGB> pca;




	obj.resize( cloudClusters.size());

	for (int i=0; i< cloudClusters.size();i++) //process cluster i
	{
		pca.setInputCloud( cloudClusters[i]);

		Eigen::Vector4f Centroid;
		Centroid = pca.getMean();
		//cout << centroid << endl;

		Eigen::Matrix3f eigenVector;
		eigenVector = pca.getEigenVectors();
		cout << "object :" << i << endl;
		cout << eigenVector << endl;
		//get the first eigenVector col
		Eigen::Vector3f orientation =eigenVector.col(0);
		cout << orientation << endl;

		//get size
		Eigen::Vector4f minP, maxP;
		pcl::getMinMax3D( *cloudClusters[i], minP, maxP);
		//cout << "X =":

		//export to obj
		obj[i].X = Centroid[0];
		obj[i].Y = Centroid[1];
		obj[i].Z = Centroid[2];
		obj[i].ax = orientation[0];
		obj[i].ay = orientation[1];
		obj[i].az = orientation[2];
	}
}

void cluster( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloudClusters)
{
	//filter out the horizontal plane
	////////////

	if (cloud->size()==0)
		return;

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud( cloud);
	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	  ec.setClusterTolerance (0.02); // 2cm
	  ec.setMinClusterSize (100);
	  ec.setMaxClusterSize (25000);
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (cloud);
	  ec.extract (cluster_indices);


	  //cloud_cluster.resize( cluster_indices.size());
	  //cout << "Size of indice "<< cloud_cluster.size()<<endl;

	  int j = 0;
	    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	    {
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(	new pcl::PointCloud<pcl::PointXYZRGB>);
	      cloudClusters.push_back( cloud_cluster);

	      int i =0;
	      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	      {
	    	  cloudClusters[j]->points.push_back (cloud->points[*pit]); //*
	    	  cloudClusters[j]->points[i].r = (j+1)*20;
	    	  cloudClusters[j]->points[i].b = (j+1)*30;
	    	  cloudClusters[j]->points[i].g = (j+1)*40;
	        i++;
	        //cloud_cluster->points.
	      }

	      cloudClusters[j]->width = cloudClusters[j]->points.size ();
	      cloudClusters[j]->height = 1;
	      cloudClusters[j]->is_dense = true;

	      //std::cout << "Call cluster \n";
	      //*cloudClustered += *cloud_cluster;

	      //*cloudClustered = *cloud_cluster;

	      std::cout << "PointCloud representing the Cluster "<<j <<" : "<< cloudClusters[j]->points.size () << " data points." << std::endl;
	      j++;
	    }

}

void transformfilter( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
			 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
{
	pcl::PassThrough<pcl::PointXYZRGB> pass;

	//filter some of them

//		pass.setInputCloud( cloud);
//		pass.setFilterFieldName( "z");
//		pass.setFilterLimits(.7, 3.0);
//		pass.filter( *cloud_filtered);

	//transform to robot base coordinates
	//i.e. transform form from the world to kinect
	//i.e. rotate about +30  +90 deg around x of world

		//tf::Transform transform;clusters
		tf::Quaternion quat;
		quat.setEuler(0, -(tiltAngle +90)*3.1416/180, 0 ); //YXZ
		//transform.setOrigin( tf::Vector3( 0, 0, 0));
		//transform.setRotation( quat);

		Eigen::Vector3f translate(.30* 2.54, -.30*2.54  ,(.44-.36)*2.54);
		Eigen::Quaternionf rotation(quat.w(), quat.x(), quat.y(), quat.z());
		pcl::transformPointCloud(*cloud, *cloud_filtered, translate, rotation);
		// 	Apply an affine transform defined by an Eigen Transform.

		//filter
		pass.setInputCloud( cloud_filtered);
		pass.setFilterFieldName( "z");
		//pass.setFilterLimits(.7, 1.5);
		pass.setFilterLimits( minZ +(float)controlMinZ/100*(maxZ-minZ)/2, maxZ -(float)controlMaxZ/100*(maxZ-minZ)/2);
		pass.filter( *cloud_filtered);

		pass.setInputCloud( cloud_filtered);
		pass.setFilterFieldName( "y");
		//pass.setFilterLimits(.0, (44-28.7)*.0254);
		pass.setFilterLimits(minY +(float)controlMinY/100*(maxY-minY)/2, maxY -(float)controlMaxY/100*(maxY-minY)/2);//(44-28.7)*.0254);
		pass.filter( *cloud_filtered);


		pass.setInputCloud( cloud_filtered);
		pass.setFilterFieldName( "x");
		pass.setFilterLimits(minX +(float)controlMinX/100*(maxX -minX)/2, maxX -(float)controlMaxX/100*(maxX-minX)/2);
		pass.filter( *cloud_filtered);

}

void transform( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromCloud,
			 pcl::PointCloud<pcl::PointXYZRGB>::Ptr toCloud)
{
	//transform to the robot coordinate
	tf::Quaternion quat;
	quat.setEuler(0, -90*3.1416/180, 0 ); //YXZ
	//transform.setOrigin( tf::Vector3( 0, 0, 0));
			//transform.setRotation( quat);

	Eigen::Vector3f translate(.30* 2.54, -.30*2.54  ,(.44-.36)*2.54);
	Eigen::Quaternionf rotation(quat.w(), quat.x(), quat.y(), quat.z());

	pcl::transformPointCloud(*fromCloud, *toCloud, translate, rotation);
			// 	Apply an affine transform defined by an Eigen Transform.
}
int main( int argc, char** argv)
{
	ros::init( argc, argv, "pcl");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

	pub = nh.advertise<sensor_msgs::PointCloud2>( "filters", 1);

	pub1 = nh.advertise<sensor_msgs::PointCloud2>( "clusters", 1);

	//pub2 = nh.advertise<roscv::ObjectPosition>( "roscv/kinect/objectPose", 1);
	pub2 = nh.advertise<roscv::ObjectPosition>( "roscv/left_hand_camera/objectPose", 1);
	//create a bunch of Slider for control
	const char WINDOW[] = "Control";

	///int
	namedWindow( WINDOW, CV_WINDOW_AUTOSIZE);
	createTrackbar( "Tilt Angle:", WINDOW, &tiltAngle, 45 );
	createTrackbar( "controlMaxZ:", WINDOW, &controlMaxZ, 100 );
	createTrackbar( "controlMinZ:", WINDOW, &controlMinZ, 100 );
	createTrackbar( "controlMaxX:", WINDOW, &controlMaxX, 100 );
	createTrackbar( "controlMinX:", WINDOW, &controlMinX, 100 );
	createTrackbar( "controlMaxY:", WINDOW, &controlMaxY, 100 );
	createTrackbar( "controlMinY:", WINDOW, &controlMinY, 100 );
	imshow(WINDOW,0);
	resizeWindow( WINDOW, 300, 400);

	ros::spin();


	return 0;
}



