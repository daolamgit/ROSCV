#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"

int main( int argc, char** argv)
{
	ros::init( argc, argv, "simple_trans_display", ros::init_options::AnonymousName) ;
	ros::NodeHandle nh;
	
	tf::TransformListener tf;
	std::string frameFrom("left_hand_camera");
	std::string frameTo("base");

	tf::StampedTransform stampTransform;
	tf.waitForTransform( frameFrom, frameTo, ros::Time(), ros::Duration(1.0));
	tf.lookupTransform( frameFrom, frameTo, ros::Time(), stampTransform);
	std::cout.precision(3);
	
	double y, p, r;

	stampTransform.getBasis().getRPY( r, p, y);
	tf::Vector3 v = stampTransform.getOrigin();
	std::cout<< "Translation : " <<v.getX() <<", " <<v.getY() << ", " <<v.getZ()<< std::endl;
	std::cout << r <<" "<<p <<" " << y <<std::endl;

}
