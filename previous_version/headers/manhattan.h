/*
 */

#pragma once

#include "cost_map.h"
#include "point.h"
#include "search.h"

class Manhattan: public Search {
 public:
	// Data
	// Methods
	Manhattan(Cost_Map* base_map, Point& start, Point& goal);
	~Manhattan();

 private:
	// Data
	// Methods
	double heuristic(const Point &pos);
};

