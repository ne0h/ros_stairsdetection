#pragma once

#include <vector>

#include <geometry_msgs/Point.h>

#include "step.hpp"
#include "stairway.hpp"

void print(Step &step);

void print(std::vector<Step> &steps);

void print(Stairway &stairway);

void print(std::vector<Stairway> &stairways);

void print(geometry_msgs::Point &p);