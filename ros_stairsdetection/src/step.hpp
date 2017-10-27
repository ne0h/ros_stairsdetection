#pragma once

#include <sstream>
#include <cmath>
#include <geometry_msgs/Point.h>

#define PI 3.14159265

/**
 * \brief Represents a single Step.
 * \author Maximilian Hess <mail@ne0h.de>
 */
class Step {

public:
	
	/**
	 * Default constructor that generates a Step with empty coordinates.
	 */
	Step() {
		m_min.x = m_min.y = m_min.z = 0.0;
		m_max.x = m_max.y = m_max.z = 0.0;
	}

	/**
	 * Creates a Step out of two points.
	 * @param min bottom left point
	 * @param max top right point
	 */
	Step(geometry_msgs::Point min, geometry_msgs::Point max) : m_min(min), m_max(max) {}

	/**
	 * Creates a new Step out of the bottom center point, the dimensions and its position within the stairway
	 * @param bottomCenter the point in the bottom center
	 * @param width the width of the Step
	 * @param height the height of the Step
	 * @param depth the depth of the Step
	 * @param i the position within the stairway, starting from the bottom
	 */
	Step(geometry_msgs::Point bottomCenter, const double width, const double height, const double depth, const int i) {
		m_min.x = bottomCenter.x + (i * depth);
		m_min.y = bottomCenter.y - (width / 2);
		m_min.z = bottomCenter.z + (i * height);

		m_max.x = bottomCenter.x + (i * depth);
		m_max.y = bottomCenter.y + (width / 2);
		m_max.z = bottomCenter.z + ((i+1) * height);
	}

	/**
	 * Default destructor.
	 */
	~Step() {}

	/**
	 * Returns the bottom left point.
	 * @return the bottom left point
	 */
	geometry_msgs::Point getMin() {
		return m_min;
	}

	/**
	 * Returns the top right point.
	 * @return the top right point
	 */
	geometry_msgs::Point getMax() {
		return m_max;
	}

	/**
	 * Resets the bottom left point.
	 * @param min the new point
	 */
	void setMin(geometry_msgs::Point min) {
		m_min = min;
	}

	/**
	 * Resets the top right point.
	 * @param may the new point
	 */
	void setMax(geometry_msgs::Point max) {
		m_max = max;
	}

	/**
	 * Resets both points.
	 * @param min the new bottom left point
	 * @param max the new top right point
	 */
	void setMinMax(geometry_msgs::Point min, geometry_msgs::Point max) {
		m_min = min;
		m_max = max;
	}

	/**
	 * Returns the width of the Step.
	 * @return the width of the Step
	 */
	double getWidth() {
		return fabs(m_max.y - m_min.y);
	}

	/**
	 * Returns the height of the Step.
	 * @return the height of the Step.
	 */
	double getHeight() {
		return fabs(m_max.z - m_min.z);
	}

	/**
	 * Returns a new point that is in the middle of top edge.
	 * @return a new point that is in the middle of top edge
	 */
	geometry_msgs::Point getCenterTop() {
		geometry_msgs::Point p;
		p.x = (m_max.x + m_min.x) / 2;
		p.y = (m_max.y + m_min.y) / 2;
		p.z = (m_max.z > m_min.z) ? m_max.z : m_min.z;

		return p;
	}

	/**
	 * Returns a new point that is in the middle of the bottom edge.
	 * @return a new point that is in the middle of the bottom edge
	 */
	geometry_msgs::Point getCenterBottom() {
		geometry_msgs::Point p;
		p.x = (m_max.x + m_min.x) / 2;
		p.y = (m_max.y + m_min.y) / 2;
		p.z = m_min.z;

		return p;
	}

	/**
	 * Returns the height above ground level.
	 * @return the height above ground level
	 */
	double getHeightAboveGround() {
		// Not deterministic if m_min or m_max is heigher...
		return (m_min.z < m_max.z) ? m_min.z : m_max.z;
	}

	/**
	 * Returns the inclination of the Step, expressed in degrees
	 * @return the inclination of the Step, expressed in degrees
	 */
	double getInclination() {
		return atan(fabs(m_min.x - m_max.x) / fabs(m_min.z - m_max.z)) * 180 / PI;
	}

	/**
	 * Returns a brief, human-readable description of the Step
	 * @return a brief, human-readable description of the Step
	 */
	std::string toString() {
		std::stringstream ss;

		ss << std::fixed << std::setprecision(3);
		ss << "Width: " << getWidth() << ", height: " << getHeight();
		ss << ", distance: " << m_min.x;
		//ss << ", inclination: " << getInclination();
		ss << ", height above ground: " << getHeightAboveGround();

		return ss.str();
	}

	/**
	 * Validates if this Step is equal to another Step
	 * @param other the other Step
	 */
	bool equals(Step &other) {
		return (m_min.x == other.getMin().x && m_min.y == other.getMin().y && m_min.z == other.getMin().z
			&& m_max.x == other.getMax().x && m_max.y == other.getMax().y && m_max.z == other.getMax().z);
	}

private:
	geometry_msgs::Point m_min;
	geometry_msgs::Point m_max;

};
