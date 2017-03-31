#ifndef PLANE_HPP
#define PLANE_HPP

#include <pcl/point_types.h>

class Plane {
public:
	Plane() {
		pcl::PointXYZ m_min(0.0, 0.0, 0.0);
		pcl::PointXYZ m_max(0.0, 0.0, 0.0);
		pcl::PointXYZ m_centerTop((m_max.x + m_min.x) / 2, (m_max.y + m_min.y) / 2, (m_max.z + m_min.z) / 2);
	}
	Plane(pcl::PointXYZ min, pcl::PointXYZ max) : m_min(min), m_max(max) {
		pcl::PointXYZ m_centerTop((m_max.x + m_min.x) / 2, (m_max.y + m_min.y) / 2, (m_max.z + m_min.z) / 2);
	}
	~Plane() {}

	pcl::PointXYZ getMin() {
		return m_min;
	}
	pcl::PointXYZ getMax() {
		return m_max;
	}
	pcl::PointXYZ getCenterTop() {
		return m_centerTop;
	}

private:
	pcl::PointXYZ m_min;
	pcl::PointXYZ m_max;
	pcl::PointXYZ m_centerTop;
};

#endif