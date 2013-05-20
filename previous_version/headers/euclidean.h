/*
 */

#pragma once

#include "cost_map.h"
#include "point.h"
#include "search.h"

class Euclidean: public Search {
 public:
	// Data
	// Methods
	Euclidean(Cost_Map* base_map, Point& start, Point& goal);
	~Euclidean();

 private:
	// Data
	// Methods
	double heuristic(const Point &pos);
};

