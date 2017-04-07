#include "ros_context.hpp"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>

#include "transform_helpers.hpp"

ROSContext::ROSContext() {

}

ROSContext::~ROSContext() {

}

//template<class MReq, class MRes>
void ROSContext::init(int argc, char **argv,
		void (*callback)(
				sensor_msgs::PointCloud2),
		bool (*exportStairs)(
				hmmwv_stairsdetection::ExportStairs::Request req,
				hmmwv_stairsdetection::ExportStairs::Response res),
		bool (*importStairs)(
				hmmwv_stairsdetection::ImportStairs::Request req,
				hmmwv_stairsdetection::ImportStairs::Response res),
		bool (*clearStairs)(
				hmmwv_stairsdetection::ClearStairs::Request req,
				hmmwv_stairsdetection::ClearStairs::Response res),
		std::vector<struct Stairs> *global_stairs) {

	/*
	 * load parameters from launch file
	 */
	std::string inputSetting;
	std::string stepsSetting;
	std::string stairsSetting;
	bool   useSampleDataSetting;
	ros::param::get("~input",  inputSetting);
	ros::param::get("~steps", stepsSetting);
	ros::param::get("~stairs", stairsSetting);

	ros::param::get("~publish_steps", m_publishStepsSetting);
	ros::param::get("~publish_stairs", m_publishStairsSetting);

	ros::param::get("~camera_height_above_ground", m_cameraHeightAboveGroundSetting);

	ros::param::get("~segmentation_iterations", m_segmentationIterationSetting);
	ros::param::get("~segmentation_threshold", m_segmentationThresholdSetting);

	ros::param::get("~max_step_width", m_maxStepWidthSetting);
	ros::param::get("~min_step_height", m_minStepHeightSetting);
	ros::param::get("~max_step_height", m_maxStepHeightSetting);

	ros::param::get("~parent_frame", m_cameraSetting);
	ros::param::get("~world_frame", m_worldFrameSetting);
	ros::param::get("~namespace", m_namespaceSetting);

	ros::param::get("~use_sample_data", useSampleDataSetting);

	ros::init(argc, argv, "stairsdetection");
	ros::NodeHandle nh;

	/*
	 * Init subscriber and listener
	 */
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(inputSetting.c_str(), 1, callback);
	m_pubSteps  = nh.advertise<visualization_msgs::MarkerArray>(stepsSetting.c_str(), 0);
	m_pubStairs = nh.advertise<visualization_msgs::MarkerArray>(stairsSetting.c_str(), 0);

	/*
	 * Init service get receive located stairs
	 */
	m_exportService = nh.advertiseService("export_stairs", exportStairs);
	m_importService = nh.advertiseService("import_stairs", importStairs);
	m_clearService  = nh.advertiseService("clear_stairs", clearStairs);

	// add test data
	if (useSampleDataSetting) {
		for (unsigned int i = 0; i < 3; i++) {
			struct Stairs s;
			Plane p1(pcl::PointXYZ(1*i, 2*i, 3*i), pcl::PointXYZ(1.5*i, 2.5*i, 3.5*i));
			Plane p2(pcl::PointXYZ(1.1*i, 2.1*i, 3.1*i), pcl::PointXYZ(1.4*i, 2.4*i, 3.4*i));
			Plane p3(pcl::PointXYZ(1.2*i, 2.2*i, 3.2*i), pcl::PointXYZ(1.3*i, 2.3*i, 3.3*i));

			s.steps.push_back(p1);
			s.steps.push_back(p2);
			s.steps.push_back(p3);

			global_stairs->push_back(s);
		}
	}

	ros::spin();
}

void ROSContext::buildRosMarkerSteps(visualization_msgs::Marker *marker, std::vector<Plane> *planes, float *color) {

	marker->header.frame_id = m_cameraSetting.c_str();
	marker->header.stamp = ros::Time::now();
	marker->ns = m_namespaceSetting.c_str();
	marker->id = 0;
	marker->lifetime = ros::Duration();

	marker->type = visualization_msgs::Marker::LINE_LIST;
	marker->action = visualization_msgs::Marker::ADD;

	marker->scale.x = 0.05f;
	marker->color.r = color[0];
	marker->color.g = color[1];
	marker->color.b = color[2];
	marker->color.a = 1.0;

	for (std::vector<Plane>::iterator it = planes->begin(); it != planes->end(); it++) {

		std::vector<pcl::PointXYZ> points;
		buildStepFromAABB(&(*it), &points);

		geometry_msgs::Point p1;
		transformPCLPointToROSPoint(&points.at(0), &p1);

		geometry_msgs::Point p2;
		transformPCLPointToROSPoint(&points.at(1), &p2);

		geometry_msgs::Point p3;
		transformPCLPointToROSPoint(&points.at(2), &p3);

		geometry_msgs::Point p4;
		transformPCLPointToROSPoint(&points.at(3), &p4);

		marker->points.push_back(p1);
		marker->points.push_back(p2);
		marker->points.push_back(p2);
		marker->points.push_back(p3);
		marker->points.push_back(p3);
		marker->points.push_back(p4);
		marker->points.push_back(p4);
		marker->points.push_back(p1);

		const float width  = fabs((*it).getMax().x - (*it).getMin().x);
		const float height = fabs((*it).getMax().y - (*it).getMin().y);
		ROS_INFO("Width: %f | Height: %f | Height above zero: %f", width, height,
			(*it).getMin().y + m_cameraHeightAboveGroundSetting);
	}
}

void ROSContext::buildROSMarkerStairs(visualization_msgs::Marker *marker, struct Stairs *stairs, float *color) {
	
	// draw front of the steps
	buildRosMarkerSteps(marker, &stairs->steps, color);

	// draw surface of the steps
	if (stairs->steps.size() > 0) {
		for (unsigned int i = 1; i < stairs->steps.size(); i++) {
			std::vector<pcl::PointXYZ> pointsCur;
			buildStepFromAABB(&stairs->steps.at(i), &pointsCur);
			geometry_msgs::Point pc1;
			transformPCLPointToROSPoint(&pointsCur.at(0), &pc1);
			geometry_msgs::Point pc2;
			transformPCLPointToROSPoint(&pointsCur.at(1), &pc2);
			geometry_msgs::Point pc3;
			transformPCLPointToROSPoint(&pointsCur.at(2), &pc3);
			geometry_msgs::Point pc4;
			transformPCLPointToROSPoint(&pointsCur.at(3), &pc4);

			std::vector<pcl::PointXYZ> pointsBefore;
			buildStepFromAABB(&stairs->steps.at(i - 1), &pointsBefore);
			geometry_msgs::Point pb1;
			transformPCLPointToROSPoint(&pointsBefore.at(0), &pb1);
			geometry_msgs::Point pb2;
			transformPCLPointToROSPoint(&pointsBefore.at(1), &pb2);
			geometry_msgs::Point pb3;
			transformPCLPointToROSPoint(&pointsBefore.at(2), &pb3);
			geometry_msgs::Point pb4;
			transformPCLPointToROSPoint(&pointsBefore.at(3), &pb4);

			/*
			 * Get vertices of the rectangle
			 *
			 *  p2-----------------p3
			 *  |                   |
			 *  |                   |
			 *  p1-----------------p4
			 *
			 */

			marker->points.push_back(pc1);
			marker->points.push_back(pb2);
			marker->points.push_back(pc4);
			marker->points.push_back(pb3);
		}
	}
}

/**
 * Shows stairs in RVIZ
 */
void ROSContext::showStairsInRVIZ(std::vector<struct Stairs> *stairs) {

	if (m_publishStairsSetting) {
		visualization_msgs::MarkerArray markerArray;

		for (std::vector<struct Stairs>::iterator it = stairs->begin(); it != stairs->end(); it++) {
			visualization_msgs::Marker marker;
			float color[3];
			color[0] = color[2] = 0.f;
			color[1] = 1.f;
			buildROSMarkerStairs(&marker, &(*it), color);
			markerArray.markers.push_back(marker);
		}

		m_pubStairs.publish(markerArray);
	}
}

void ROSContext::publishSteps(std::vector<Plane> *planes) {
	visualization_msgs::MarkerArray markerArray;
	visualization_msgs::Marker marker;
	float color[3];
	color[0] = color[1] = 0.f;
	color[2] = 1.f;

	buildRosMarkerSteps(&marker, planes, color);
	markerArray.markers.push_back(marker);
	m_pubSteps.publish(markerArray);
}
