#include <vector>

#include "transform_helper.hpp"
#include "plane.hpp"

void TransformHelper::getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Plane *plane) {
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	pcl::PointXYZ min, max;
	feature_extractor.getAABB(min, max);
	plane->setMinMax(min, max);
}

void TransformHelper::transformStairsToWorldCoordinates(struct Stairs *stairs) {

	for (std::vector<Plane>::iterator it = stairs->steps.begin(); it != stairs->steps.end(); it++) {
		transformToWorldCoordinates(&(*it));
	}
}

bool TransformHelper::transformToWorldCoordinates(pcl::PointXYZ *point) {
	
	geometry_msgs::Point tmp;
	transformPCLPointToROSPoint(point, &tmp);
	if (!transformToWorldCoordinates(&tmp)) {
		return false;
	}
	transformROSPointToPCLPoint(&tmp, point);
	return true;
}

bool TransformHelper::transformToWorldCoordinates(geometry_msgs::Point *point) {

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped ts;
	try {
		ts = tfBuffer.lookupTransform(m_cameraSetting.c_str(), m_worldFrameSetting.c_str(), ros::Time(0));
	} catch (tf2::TransformException &ex) {
		ROS_WARN("Failed to transform to world coordinates. World frame id is '%s':", m_worldFrameSetting.c_str());
		ROS_WARN("%s", ex.what());
		return false;
	}

	point->x = point->x + ts.transform.translation.x;
	point->y = point->y + ts.transform.translation.z;
	point->z = point->z + ts.transform.translation.z;

	return true;
}

bool TransformHelper::transformToWorldCoordinates(Plane *plane) {

	pcl::PointXYZ min = plane->getMin();
	pcl::PointXYZ max = plane->getMax();
	transformToWorldCoordinates(&min);
	transformToWorldCoordinates(&max);

	plane->setMinMax(min, max);
	return true;
}

void TransformHelper::transformPCLPointToROSPoint(pcl::PointXYZ *input, geometry_msgs::Point *output) {
	output->x = input->z;
	output->y = input->x * (-1.f);
	output->z = input->y * (-1.f);
}

void TransformHelper::transformROSPointToPCLPoint(geometry_msgs::Point *input, pcl::PointXYZ *output) {
	output->x = input->y * (-1.f);
	output->y = input->z * (-1.f);
	output->z = input->x;
}

bool TransformHelper::transformToBaseLinkCoordinates(geometry_msgs::Point *point) {

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped ts;
	try {
		ts = tfBuffer.lookupTransform(m_cameraSetting.c_str(), m_worldFrameSetting.c_str(), ros::Time(0));
	} catch (tf2::TransformException &ex) {
		ROS_WARN("Failed to transform to world coordinates. World frame id is %s", m_worldFrameSetting.c_str());
		ROS_WARN("%s", ex.what());
		return false;
	}

	point->x = point->x - ts.transform.translation.x;
	point->y = point->y - ts.transform.translation.z;
	point->z = point->z - ts.transform.translation.z;

	return true;
}

void TransformHelper::buildStepFromAABB(Plane *plane, std::vector<pcl::PointXYZ> *points) {
	points->push_back(plane->getMin());
	points->push_back(pcl::PointXYZ(plane->getMin().x, plane->getMax().y, plane->getMin().z));
	points->push_back(plane->getMax());
	points->push_back(pcl::PointXYZ(plane->getMax().x, plane->getMin().y, plane->getMax().z));
}