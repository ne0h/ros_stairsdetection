#pragma once

#include <vector>
#include "plane.hpp"

class Stairway {

public:

    std::vector<Plane>& getSteps() {
        return m_steps;
    }

private:
    std::vector<Plane> m_steps;

};