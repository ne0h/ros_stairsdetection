#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
	ROS_INFO("Callback! %d %d", cloud->width, cloud->height);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "stairwaydetector");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback);
	ros::spin();
	
	return 0;
}
