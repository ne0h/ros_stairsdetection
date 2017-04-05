#include <vector>

#include "transform_helpers.hpp"
#include "plane.hpp"

void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Plane *plane) {
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	pcl::PointXYZ min, max;
	feature_extractor.getAABB(min, max);
	plane->setMinMax(min, max);
}

void transformStairsToWorldCoordinates(struct Stairs *stairs, std::string cameraSetting,
		std::string worldFrameSetting) {

	for (std::vector<Plane>::iterator it = stairs->steps.begin(); it != stairs->steps.end(); it++) {

		pcl::PointXYZ min = (*it).getMin();
		pcl::PointXYZ max = (*it).getMax();
		transformToWorldCoordinates(&min, cameraSetting, worldFrameSetting);
		transformToWorldCoordinates(&max, cameraSetting, worldFrameSetting);

		(*it).setMinMax(min, max);
	}
}

bool transformToWorldCoordinates(pcl::PointXYZ *p, std::string cameraSetting, std::string worldFrameSetting) {
	geometry_msgs::Point tmp;
	transformPCLPointToROSPoint(p, &tmp);
	if (!transformToWorldCoordinates(&tmp, cameraSetting, worldFrameSetting))
		return false;
	transformROSPointToPCLPoint(&tmp, p);
	return true;
}

bool transformToWorldCoordinates(geometry_msgs::Point *p, std::string cameraSetting, std::string worldFrameSetting) {
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped ts;
	try {
		ts = tfBuffer.lookupTransform(cameraSetting.c_str(), worldFrameSetting.c_str(), ros::Time(0));
	} catch (tf2::TransformException &ex) {
		ROS_WARN("Failed to transform to world coordinates. World frame id is '%s'", worldFrameSetting.c_str());
		ROS_WARN("%s", ex.what());
		return false;
	}

	p->x = p->x + ts.transform.translation.x;
	p->y = p->y + ts.transform.translation.z;
	p->z = p->z + ts.transform.translation.z;

	return true;
}

void transformPCLPointToROSPoint(pcl::PointXYZ *input, geometry_msgs::Point *output) {
	output->x = input->z;
	output->y = input->x * (-1.f);
	output->z = input->y * (-1.f);
}

void transformROSPointToPCLPoint(geometry_msgs::Point *input, pcl::PointXYZ *output) {
	output->x = input->y * (-1.f);
	output->y = input->z * (-1.f);
	output->z = input->x;
}

bool transformToBaseLinkCoordinates(geometry_msgs::Point *p, std::string cameraSetting, std::string worldFrameSetting) {
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped ts;
	try {
		ts = tfBuffer.lookupTransform(cameraSetting.c_str(), worldFrameSetting.c_str(), ros::Time(0));
	} catch (tf2::TransformException &ex) {
		ROS_WARN("Failed to transform to world coordinates. World frame id is %s", worldFrameSetting.c_str());
		ROS_WARN("%s", ex.what());
		return false;
	}

	p->x = p->x - ts.transform.translation.x;
	p->y = p->y - ts.transform.translation.z;
	p->z = p->z - ts.transform.translation.z;

	return true;
}