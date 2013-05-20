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

class Boundaries_Blocking: public Search {
 public:
	// Data
	int coarse_expanded;

	// Methods
	Boundaries_Blocking(Cost_Map* base_map, Point& start, Point& goal, unsigned short nlevels, unsigned short xs, unsigned short ys);
	~Boundaries_Blocking();

 private:
	// Data
	Heap** fringes;

	// Methods
	double heuristic(const Point &pos);
	void fill_levels();
	void coarse_search(Node* target);
	bool next_upper_successor(const Point &pos, Point &next);
	int next_upper_successor_state;
};

