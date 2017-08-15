#ifndef PLANE_HPP
#define PLANE_HPP

#include <sstream>
#include <pcl/point_types.h>

class Plane {
public:
	Plane(pcl::PointXYZ min, pcl::PointXYZ max) : m_min(min), m_max(max) {
		init();
	}

	Plane() : m_min(pcl::PointXYZ(0.0, 0.0, 0.0)), m_max(pcl::PointXYZ(0.0, 0.0, 0.0)) {
		init();
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

	void calculateCenterTop() {

	}

	std::string toString() {
		std::stringstream ss;
		ss << "Min: (" << m_min.x << ", " << m_min.y << ", " << m_min.z << ") | ";
		ss << "Max: (" << m_max.x << ", " << m_max.y << ", " << m_max.z << ") x Max (";
		return ss.str();
	}

	bool equals(struct Plane other) {
		return (m_min.x == other.getMin().x && m_min.y == other.getMin().y && m_min.z == other.getMin().z
			&& m_max.x == other.getMax().x && m_max.y == other.getMax().y && m_max.z == other.getMax().z);
	}

private:
	pcl::PointXYZ m_min;
	pcl::PointXYZ m_max;
	pcl::PointXYZ m_centerTop;

	void init() {
		m_centerTop = pcl::PointXYZ((m_max.x + m_min.x) / 2, (m_max.y + m_min.y) / 2, (m_max.z + m_min.z) / 2);
	}
};

#endif