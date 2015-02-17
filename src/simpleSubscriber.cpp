#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"
void displayCallback( const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	//ROS_INFO(" I receive : [%s]", msg->distortion_model.c_str());
	std::cout << *msg <<std::endl;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "display");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/cameras/left_hand_camera/camera_info", 1, displayCallback);
	//ros::Subscriber sub = n.subscribe("/distortedCenters", 1, displayCallback);
	ros::spin();
	return 0;
  	
}
