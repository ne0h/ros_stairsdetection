#ifndef PLANE_HPP
#define PLANE_HPP

#include <sstream>
#include <cmath>
#include <pcl/point_types.h>

/**
 * @file plane.hpp
 * @brief A three-dimensional plane.
 * @author Maximilian Hess <mail@ne0h.de>
 *
 * This class is adjusted to fullfil the needs of stairsdetection. In this context, Planes normally are vertical in
 * front of the robot.
 */
class Plane {

public:
	
	/**
	 * Default constructor that generates a plane with empty coordinates.
	 */
	Plane() : m_min(pcl::PointXYZ(0.0, 0.0, 0.0)), m_max(pcl::PointXYZ(0.0, 0.0, 0.0)) {}

	/**
	 * Creates a plane out of two points.
	 * @param min bottom left point
	 * @param max top right point
	 */
	Plane(pcl::PointXYZ min, pcl::PointXYZ max) : m_min(min), m_max(max) {}

	/**
	 * Default destructor.
	 */
	~Plane() {}

	/**
	 * Returns the bottom left point.
	 * @return the bottom left point
	 */
	pcl::PointXYZ getMin() {
		return m_min;
	}

	/**
	 * Returns the top right point.
	 * @return the top right point
	 */
	pcl::PointXYZ getMax() {
		return m_max;
	}

	/**
	 * Resets the bottom left point.
	 * @param min the new point
	 */
	void setMin(pcl::PointXYZ min) {
		m_min = min;
	}

	/**
	 * Resets the top right point.
	 * @param may the new point
	 */
	void setMax(pcl::PointXYZ max) {
		m_max = max;
	}

	/**
	 * Resets both points.
	 * @param min the new bottom left point
	 * @param max the new top right point
	 */
	void setMinMax(pcl::PointXYZ min, pcl::PointXYZ max) {
		m_min = min;
		m_max = max;
	}

	/**
	 * Returns the width of the Plane.
	 * @return the width of the Plane
	 */
	float getWidth() {
		return fabs(m_max.x - m_min.x);
	}

	/**
	 * Returns the height of the Plane.
	 * @return the height of the Plane.
	 */
	float getHeight() {
		return fabs(m_max.y - m_min.y);
	}

	/**
	 * Returns a new point that is in the middle of top edge.
	 * @return a new point that is in the middle of top edge
	 */
	pcl::PointXYZ getCenterTop() {
		pcl::PointXYZ(m_max.x, (m_max.y + m_min.y) / 2, (m_max.z + m_min.z) / 2);
	}

	/**
	 * Returns a new point that is in the middle of the bottom edge.
	 * @return a new point that is in the middle of the bottom edge
	 */
	pcl::PointXYZ getCenterBottom() {
		return pcl::PointXYZ(m_min.x, (m_max.y + m_min.y) / 2, (m_max.z + m_min.z) / 2);
	}

	/**
	 * Returns the height above ground level.
	 * @return the height above ground level
	 */
	float getHeightAboveGround() {
		return m_min.y;
	}

	/**
	 * Returns a brief, human-readable description of the plane
	 * @return a brief, human-readable description of the plane
	 */
	std::string toString() {
		pcl::PointXYZ p = getCenterBottom();
		std::stringstream ss;

		ss << "Width: " << getWidth() << ", height: " << getHeight();
		ss << ", height above ground: " << getHeightAboveGround() << "\"                                   ";
		ss << " at (" << p.x << " | " << p.y << " | " << p.z << ")";

		return ss.str();
	}

	/**
	 * Validates if this Plane is equal with another Plane
	 * @param other the other Plane
	 */
	bool equals(struct Plane other) {
		return (m_min.x == other.getMin().x && m_min.y == other.getMin().y && m_min.z == other.getMin().z
			&& m_max.x == other.getMax().x && m_max.y == other.getMax().y && m_max.z == other.getMax().z);
	}

private:
	pcl::PointXYZ m_min;
	pcl::PointXYZ m_max;
};

#endif