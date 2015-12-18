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
ros::Publisher pub0c;
ros::Publisher pub1c;
ros::Publisher pub2c;
ros::Publisher pub3c;
ros::Publisher pub4c;
ros::Publisher pub5c;
ros::Publisher pub0p;
ros::Publisher pub1p;
ros::Publisher pub2p;
ros::Publisher pub3p;
ros::Publisher pub4p;
ros::Publisher pub5p;

struct Plane {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointXYZ min;
	pcl::PointXYZ max;
};

//void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ* min_point_AABB, pcl::PointXYZ* max_point_AABB) {
void getAABB(struct Plane *plane) {
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(plane->cloud);
	feature_extractor.compute();
	feature_extractor.getAABB(plane->min, plane->max);
}

void buildRosMessage(geometry_msgs::PolygonStamped *output, struct Plane *plane) {
	output->header.frame_id = "/base_link";

	geometry_msgs::Point32 p1;
	p1.x = plane->min.z;
	p1.y = plane->min.x * (-1);
	p1.z = plane->min.y * (-1);

	geometry_msgs::Point32 p2;
	p2.x = plane->min.z;
	p2.y = plane->max.x * (-1);
	p2.z = plane->min.y * (-1);

	geometry_msgs::Point32 p3;
	p3.x = plane->min.z;
	p3.y = plane->max.x * (-1);
	p3.z = plane->max.y * (-1);

	geometry_msgs::Point32 p4;
	p4.x = plane->min.z;
	p4.y = plane->min.x * (-1);
	p4.z = plane->max.y * (-1);

	output->polygon.points.push_back(p1);
	output->polygon.points.push_back(p2);
	output->polygon.points.push_back(p3);
	output->polygon.points.push_back(p4);
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input) {
	ROS_INFO("Callback!");

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
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes0;

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

		extract.setNegative(true);
		extract.filter(*cloud2);
		cloud.swap(cloud2);

		planes0.push_back(cloud1);
		i++;
	}

	std::vector<struct Plane> planes1;

	for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = planes0.begin(); it != planes0.end(); it++) {
		struct Plane plane;
		plane.cloud = *it;
		getAABB(&plane);

		planes1.push_back(plane);
	}

	geometry_msgs::PolygonStamped output0p;
	buildRosMessage(&output0p, &planes1.at(0));
	pub0p.publish(output0p);

	sensor_msgs::PointCloud2 output0c;
	pcl::toROSMsg(*planes0.at(0), output0c);
	pub0c.publish(output0c);

	if (planes1.size() > 1) {
		geometry_msgs::PolygonStamped output1p;
		buildRosMessage(&output1p, &planes1.at(1));
		pub1p.publish(output1p);

		sensor_msgs::PointCloud2 output1c;
		pcl::toROSMsg(*planes0.at(1), output1c);
		pub1c.publish(output1c);
	}

	if (planes1.size() > 2) {
		geometry_msgs::PolygonStamped output2p;
		buildRosMessage(&output2p, &planes1.at(2));
		pub2p.publish(output2p);

		sensor_msgs::PointCloud2 output2c;
		pcl::toROSMsg(*planes0.at(2), output2c);
		pub2c.publish(output0c);
	}

	if (planes1.size() > 3) {
		geometry_msgs::PolygonStamped output3p;
		buildRosMessage(&output3p, &planes1.at(3));
		pub3p.publish(output3p);

		sensor_msgs::PointCloud2 output3c;
		pcl::toROSMsg(*planes0.at(3), output3c);
		pub3c.publish(output0c);
	}

	if (planes1.size() > 4) {
		geometry_msgs::PolygonStamped output4p;
		buildRosMessage(&output4p, &planes1.at(4));
		pub4p.publish(output4p);

		sensor_msgs::PointCloud2 output4c;
		pcl::toROSMsg(*planes0.at(4), output4c);
		pub4c.publish(output4c);
	}

	if (planes1.size() > 5) {
		geometry_msgs::PolygonStamped output5p;
		buildRosMessage(&output5p, &planes1.at(5));
		pub5p.publish(output5p);

		sensor_msgs::PointCloud2 output5c;
		pcl::toROSMsg(*planes0.at(5), output5c);
		pub5c.publish(output5c);
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "stairwaydetector");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback);
	pub0c = nh.advertise<sensor_msgs::PointCloud2>("/fuckingoutput0", 1);
	pub1c = nh.advertise<sensor_msgs::PointCloud2>("/fuckingoutput1", 1);
	pub2c = nh.advertise<sensor_msgs::PointCloud2>("/fuckingoutput2", 1);
	pub3c = nh.advertise<sensor_msgs::PointCloud2>("/fuckingoutput3", 1);
	pub4c = nh.advertise<sensor_msgs::PointCloud2>("/fuckingoutput4", 1);
	pub5c = nh.advertise<sensor_msgs::PointCloud2>("/fuckingoutput5", 1);
	pub0p = nh.advertise<geometry_msgs::PolygonStamped>("/step0", 1);
	pub1p = nh.advertise<geometry_msgs::PolygonStamped>("/step1", 1);
	pub2p = nh.advertise<geometry_msgs::PolygonStamped>("/step2", 1);
	pub3p = nh.advertise<geometry_msgs::PolygonStamped>("/step3", 1);
	pub4p = nh.advertise<geometry_msgs::PolygonStamped>("/step4", 1);
	pub5p = nh.advertise<geometry_msgs::PolygonStamped>("/step5", 1);
	ros::spin();
	
	return 0;
}
