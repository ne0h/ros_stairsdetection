#ifndef PLANE_HPP
#define PLANE_HPP

#include <pcl/point_types.h>

class Plane {
public:
	Plane(pcl::PointXYZ min, pcl::PointXYZ max) : m_min(min), m_max(max) {
		m_centerTop = pcl::PointXYZ((m_max.x + m_min.x) / 2, (m_max.y + m_min.y) / 2, (m_max.z + m_min.z) / 2);
	}

	Plane() : Plane(pcl::PointXYZ(0.0, 0.0, 0.0), pcl::PointXYZ(0.0, 0.0, 0.0)) {}

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

	void setMin(pcl::PointXYZ min) {
		m_min = min;
	}

	void setMax(pcl::PointXYZ max) {
		m_max = max;
	}

	void setMinMax(pcl::PointXYZ min, pcl::PointXYZ max) {
		m_min = min;
		m_max = max;
	}

private:
	pcl::PointXYZ m_min;
	pcl::PointXYZ m_max;
	pcl::PointXYZ m_centerTop;
};

#endif