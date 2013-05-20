/*
 */

#pragma once

#include "cost_map.h"
#include "point.h"
#include "search.h"

class Octile: public Search {
 public:
	// Data
	// Methods
	Octile(Cost_Map* base_map, Point& start, Point& goal);
	~Octile();

 private:
	// Data
	// Methods
	double heuristic(const Point &pos);
};

