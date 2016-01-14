#include <vector>
#include <cmath>
#include <string>

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

int   segmentationIterationSetting;
float segmentationThresholdSetting;

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

/**
 * Transforms a point from PCL coordinate system to ROS coordinate system
 *
 * Documentation:
 * ROS: http://wiki.ros.org/geometry/CoordinateFrameConventions
 */
void transformPCLPointToROSPoint(pcl::PointXYZ *input, geometry_msgs::Point *output) {
	output->x = input->z;
	output->y = input->x * (-1.f);
	output->z = input->y * (-1.f);
}

void buildRosMarker(visualization_msgs::Marker *marker, struct Plane *plane, unsigned int id) {
	string cameraSetting;
	string namespaceSetting;
	ros::param::get("~parent_frame", cameraSetting);
	ros::param::get("~namespace", namespaceSetting);

	marker->header.frame_id = cameraSetting.c_str();
	marker->header.stamp = ros::Time::now();
	marker->ns = namespaceSetting.c_str();
	marker->id = id;
	marker->lifetime = ros::Duration();

	marker->type = visualization_msgs::Marker::LINE_STRIP;
	marker->action = visualization_msgs::Marker::ADD;

	marker->scale.x = 0.05f;

	// generate random color to separate rectangles from each other
	marker->color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	marker->color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	marker->color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	marker->color.a = 1.0;

	/*
	 * Get vertices of the rectangle and transform them to ROS coordinates
	 *
	 *  p2-----------------p3
	 *  |                   |
	 *  |                   |
	 *  p1-----------------p4
	 *
	 */

	geometry_msgs::Point p1;
	transformPCLPointToROSPoint(&plane->min, &p1);

	geometry_msgs::Point p2;
	pcl::PointXYZ leftTop(plane->min.x, plane->max.y, plane->min.z);
	transformPCLPointToROSPoint(&leftTop, &p2);

	geometry_msgs::Point p3;
	transformPCLPointToROSPoint(&plane->max, &p3);

	geometry_msgs::Point p4;
	pcl::PointXYZ rightBottom(plane->max.x, plane->min.y, plane->max.z);
	transformPCLPointToROSPoint(&rightBottom, &p4);

	marker->points.push_back(p1);
	marker->points.push_back(p2);
	marker->points.push_back(p3);
	marker->points.push_back(p4);
	marker->points.push_back(p1);
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input) {
	ROS_INFO("=================================================================");

	// convert from ros::pointcloud2 to pcl::pointcloud2
	pcl::PCLPointCloud2* unfilteredCloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr unfilteredCloudPtr(unfilteredCloud);
	pcl_conversions::toPCL(*input, *unfilteredCloud);

	// downsample the input data to speed things up.
	pcl::PCLPointCloud2::Ptr filteredCloud(new pcl::PCLPointCloud2);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(unfilteredCloudPtr);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*filteredCloud);

	// convert to pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*filteredCloud, *cloud);

	// Do the parametric segmentation
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(segmentationIterationSetting);
	seg.setDistanceThreshold(segmentationThresholdSetting);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),
			cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	unsigned int pointsAtStart = cloud->points.size(), id = -1;

	std::vector<struct Plane> planes;
	visualization_msgs::MarkerArray markerArray;

	// Extract a model and repeat while 0.5% of the original cloud is still present
	while (cloud->points.size() > 0.005 * pointsAtStart) {
		id++;

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

		const float width  = fabs(plane.max.x - plane.min.x);
		const float height = fabs(plane.max.y - plane.min.y);
		//ROS_INFO("Width: %f | Height: %f", width, height);

		// Remove planes with less than 5cm or more than 40cm and remove rectangles with less than 40cm
		if (height > 0.4f || height < 0.05f || width < 0.4f) {
			continue;
		}

		// Remove rectangle that are not (more or less) orthogonal to the robot
		const float depthDiff = fabs(plane.max.z - plane.min.z);
		const float depthThreshold = 0.2f;
		if (depthDiff > depthThreshold) {
			continue;
		}
		
		visualization_msgs::Marker marker;
		buildRosMarker(&marker, &plane, id);
		markerArray.markers.push_back(marker);

		ROS_INFO("Width: %f | Height: %f | Height above zero: %f", width, height, plane.min.y);
	}

    pub.publish(markerArray);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "stairwaydetector");
	ros::NodeHandle nh;

	/*
	 * load parameters from launch file
	 */
	string inputSetting;
	string outputSetting;
	ros::param::get("~input",  inputSetting);
	ros::param::get("~output", outputSetting);

	ros::param::get("~segmentation_iterations", segmentationIterationSetting);
	ros::param::get("~segmentation_threshold", segmentationThresholdSetting);
	
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(inputSetting.c_str(), 1, callback);
	pub = nh.advertise<visualization_msgs::MarkerArray>(outputSetting.c_str(), 0);

	ros::spin();
	
	return 0;
}
