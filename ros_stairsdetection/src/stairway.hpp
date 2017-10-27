#pragma once

#include <vector>
#include "step.hpp"
#include <ros/ros.h>

/**
 * \brief Holds a single Stairway.
 *
 * A Stairway consists of a std::vector of Steps, starting from below. Each Stairway holds at least one Step as a
 * minimum.
 */
class Stairway {

public:

    /**
     * Returns a std::vector of Steps starting from below.
     * @return a std::vector of Steps starting from below.
     */
    std::vector<Step>& getSteps() {
        return m_steps;
    }

    /**
     * Returns the first Step of the Stairway.
     * @return the first Step of the Stairway.
     */
    Step& getFirst() {
        return m_steps.front();
    }

    bool almostEquals(Stairway &other) {
        const double threshold = 0.1;

        ROS_INFO("#######################################");
        ROS_INFO("%f vs %f", getFirst().getCenterBottom().x, other.getFirst().getCenterBottom().x);
        ROS_INFO("%f vs %f", getFirst().getCenterBottom().y, other.getFirst().getCenterBottom().y);
        ROS_INFO("%f vs %f", getFirst().getCenterBottom().z, other.getFirst().getCenterBottom().z);

        return (fabs(getFirst().getCenterBottom().x - other.getFirst().getCenterBottom().x) < threshold
            &&  fabs(getFirst().getCenterBottom().y - other.getFirst().getCenterBottom().y) < threshold
            &&  fabs(getFirst().getCenterBottom().z - other.getFirst().getCenterBottom().z) < threshold
        );
	}

private:
    std::vector<Step> m_steps;

};
