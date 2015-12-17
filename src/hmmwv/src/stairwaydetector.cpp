#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std;
ros::Publisher pub;

void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ* min_point_AABB, pcl::PointXYZ* max_point_AABB) {
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getAABB(*min_point_AABB, *max_point_AABB);
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input) {
	ROS_INFO("Callback! %d %d", input->width, input->height);

	// convert from ros::pointcloud2 to pcl::pointcloud2
	pcl::PCLPointCloud2* unfilteredCloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr unfilteredCloudPtr(unfilteredCloud);
	pcl_conversions::toPCL(*input, *unfilteredCloud);

	// create a voxelgrid to downsample the input data to speed things up.
	pcl::PCLPointCloud2::Ptr filteredCloud (new pcl::PCLPointCloud2);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(unfilteredCloudPtr);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*filteredCloud);

	// convert to pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*filteredCloud, *cloud);

	// Does the parametric segmentation
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),
		cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	unsigned int i = 0, pointsAtStart = cloud->points.size();

	// while 30% of the original cloud is still present
	while (cloud->points.size() > 0.3 * pointsAtStart) {

		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0) {
			ROS_WARN("Could not estimate a planar model for the given dataset.");
			break;
		}

		// extract the inliers
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud1);
		/*std::cerr << "PointCloud representing the planar component: " << cloud1->width * cloud1->height
			<< " data points." << std::endl;
		std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;*/

		extract.setNegative(true);
		extract.filter(*cloud2);
		cloud.swap(cloud2);

		planes.push_back(cloud1);
		i++;
	}

	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	getAABB(planes.at(4), &min_point_AABB, &max_point_AABB);

	geometry_msgs::PolygonStamped output;
	output.header.frame_id = "/base_link";

	geometry_msgs::Point32 p1;
	p1.x = min_point_AABB.z;
	p1.y = min_point_AABB.x;
	p1.z = min_point_AABB.y * (-1);

	geometry_msgs::Point32 p2;
	p2.x = min_point_AABB.z;
	p2.y = max_point_AABB.x;
	p2.z = min_point_AABB.y * (-1);

	geometry_msgs::Point32 p3;
	p3.x = min_point_AABB.z;
	p3.y = max_point_AABB.x;
	p3.z = max_point_AABB.y * (-1);

	geometry_msgs::Point32 p4;
	p4.x = min_point_AABB.z;
	p4.y = min_point_AABB.x;
	p4.z = max_point_AABB.y * (-1);

	output.polygon.points.push_back(p1);
	output.polygon.points.push_back(p2);
	output.polygon.points.push_back(p3);
	output.polygon.points.push_back(p4);

	pub.publish(output);

	/*sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*planes.at(5), output);
	pub.publish(output);*/
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "stairwaydetector");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback);
	//pub = nh.advertise<sensor_msgs::PointCloud2>("/fuckingoutput", 1);
	pub = nh.advertise<geometry_msgs::PolygonStamped>("/fuckingoutput", 1);
	ros::spin();
	
	return 0;
}
