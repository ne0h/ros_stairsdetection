#include <vector>

#include "transform_helper.hpp"
#include "plane.hpp"
#include "print_helpers.hpp"

void TransformHelper::getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Plane &plane) {
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	pcl::PointXYZ min, max;
	feature_extractor.getAABB(min, max);

	// transform PCL points to ROS points
	geometry_msgs::Point min_r, max_r;
	transformPCLPointToROSPoint(min, min_r);
	transformPCLPointToROSPoint(max, max_r);

	plane.setMinMax(max_r, min_r);
}

bool TransformHelper::transform(geometry_msgs::Point &point, std::string &target_frame, std::string &source_frame,
		float factor) {

	// Make sure that the transform buffer is not NULL
	if (m_tfBuffer == NULL) {
		ROS_ERROR("Transformlistener is NULL");
		return false;
	}

	geometry_msgs::TransformStamped ts;
	try {
		ts = m_tfBuffer->lookupTransform(target_frame.c_str(), source_frame.c_str(), ros::Time(0));
	} catch (tf2::TransformException &ex) {
		ROS_WARN("Failed to transform '%s' -> '%s'", target_frame.c_str(), source_frame.c_str());
		ROS_WARN("%s", ex.what());
		return false;
	}

	point.x = point.x + ts.transform.translation.x * factor;
	point.y = point.y + ts.transform.translation.y * factor;
	point.z = point.z + ts.transform.translation.z * factor;

	return true;
}

bool TransformHelper::transformToWorldCoordinates(Plane &plane) {
	geometry_msgs::Point min = plane.getMin();
	geometry_msgs::Point max = plane.getMax();
	transformToWorldCoordinates(min);
	transformToWorldCoordinates(max);

	plane.setMinMax(min, max);
	return true;
}

void TransformHelper::transformPCLPointToROSPoint(pcl::PointXYZ &input, geometry_msgs::Point &output) {
	output.x = input.z;
	output.y = input.x;
	output.z = input.y * (-1.f);
}

/*
 * Get vertices of the rectangle
 *
 *  p2-----------------p3
 *  |                   |
 *  |                   |
 *  p1-----------------p4
 *
 */
void TransformHelper::buildStepFromAABB(Plane &plane, std::vector<geometry_msgs::Point> &points) {

	// p1
	points.push_back(plane.getMin());

	// p2
	geometry_msgs::Point p2;
	p2.x = plane.getMin().x;
	p2.y = plane.getMax().y;
	p2.z = plane.getMin().z;
	points.push_back(p2);

	// p3
	points.push_back(plane.getMax());

	// p4
	geometry_msgs::Point p4;
	p4.x = plane.getMax().x;
	p4.y = plane.getMin().y;
	p4.z = plane.getMax().z;
	points.push_back(p4);

	geometry_msgs::Point p1 = plane.getMin();
	geometry_msgs::Point p3 = plane.getMax();
}