#pragma once

#include <vector>

#include <geometry_msgs/Point.h>

#include "plane.hpp"
#include "stairway.hpp"

void print(Plane &plane);

void print(std::vector<Plane> &planes);

void print(Stairway &stairway);

void print(std::vector<Stairway> &stairways);

void print(geometry_msgs::Point &p);