#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std;
ros::Publisher pub;

struct Plane {
	pcl::PointXYZ min;
	pcl::PointXYZ max;
};

void printROSPoint(geometry_msgs::Point *p) {
	ROS_INFO("Point: %f %f %f", p->x, p->y, p->z);
}

void printPlane(struct Plane *plane) {
	ROS_INFO("AABB: %f %f %f -> %f %f %f", plane->min.x, plane->min.y, plane->min.z, plane->max.x, plane->max.z,
		plane->max.z);
}

void printPoint(pcl::PointXYZ *p) {
	ROS_INFO("Point: %f %f %f", p->x, p->y, p->z);
}

void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, struct Plane *plane) {
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getAABB(plane->min, plane->max);
}

void buildRosMarker(visualization_msgs::Marker *marker, struct Plane *plane, unsigned int id) {
	marker->header.frame_id = "base_link";
	marker->header.stamp = ros::Time::now();
	marker->ns = "hmmwv";
	marker->id = id;

	marker->type = visualization_msgs::Marker::LINE_STRIP;
	marker->action = visualization_msgs::Marker::ADD;

	marker->scale.x = 0.1f;

	marker->color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	marker->color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	marker->color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	marker->color.a = 1.0;

	marker->lifetime = ros::Duration();

	geometry_msgs::Point p1;
	p1.x = plane->min.z;
	p1.y = plane->min.x * (-1);
	p1.z = plane->min.y * (-1);

	geometry_msgs::Point p2;
	p2.x = plane->min.z;
	p2.y = plane->max.x * (-1);
	p2.z = plane->min.y * (-1);

	geometry_msgs::Point p3;
	p3.x = plane->min.z;
	p3.y = plane->max.x * (-1);
	p3.z = plane->max.y * (-1);

	geometry_msgs::Point p4;
	p4.x = plane->min.z;
	p4.y = plane->min.x * (-1);
	p4.z = plane->max.y * (-1);

	marker->points.push_back(p1);
	marker->points.push_back(p2);
	marker->points.push_back(p3);
	marker->points.push_back(p4);
	marker->points.push_back(p1);
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),
		cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	unsigned int pointsAtStart = cloud->points.size(), id = 0;

	std::vector<struct Plane> planes;
	visualization_msgs::MarkerArray markerArray;

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

		// calculate AABB and add to planes list
		struct Plane plane;
		getAABB(cloud1, &plane);
		printPlane(&plane);

		visualization_msgs::Marker marker;
		buildRosMarker(&marker, &plane, id);
		markerArray.markers.push_back(marker);

		id++;
	}

    pub.publish(markerArray);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "stairwaydetector");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback);
	pub = nh.advertise<visualization_msgs::MarkerArray>("/hmmwv/steps", 0);

	ros::spin();
	
	return 0;
}
