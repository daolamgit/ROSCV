/*
 * pcd_write.cpp
 *
 *  Created on: Jul 22, 2013
 *      Author: dale
 */
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>


ros::Publisher pub;
void cloud_cb( const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud< pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*input, cloud);

	pcl::ModelCoefficients coefficients;
	pcl::PointIndices inliners;

	pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setModelType( pcl::SACMODEL_PLANE);
	seg.setMethodType( pcl::SAC_RANSAC);
	seg.setDistanceThreshold( .01);
	seg.setInputCloud(( cloud.makeShared()));
	seg.segment(inliners, coefficients);

	pub.publish (coefficients);
	//cloud = input->data;
	//std::cout << "Cloud callback :" << input->width << " "<<" height :" << input->height <<std::endl;
}

int main( int argc, char** argv)
{
	ros::init( argc, argv, "plc");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

	pub = nh.advertise<pcl::ModelCoefficients>( "outputs", 1);

	ros::spin();


	return 0;
}



