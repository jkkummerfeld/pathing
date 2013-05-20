/*
 */

#pragma once

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include "node.h"
#include "path.h"
#include "cost_map.h"
#include "heap.h"
#include "search.h"

class UCS: public Search {
 public:
	// Data
	// Methods
	UCS(Cost_Map* base_map, Point& start, Point& goal);
	~UCS();

 private:
	// Data

	// Methods
	double heuristic(const Point &pos);
};

