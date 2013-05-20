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

class CUCS_Heuristic: public Search {
 public:
	// Data
	int coarse_expanded;

	// Methods
	CUCS_Heuristic(Cost_Map* base_map, Point& start, Point& goal);
	~CUCS_Heuristic();

 private:
	// Data
	Heap upper_fringe;
	Point upper_goal;
	Point upper_start;

	// Methods
	double heuristic(const Point &pos);

	void coarse_search(Node* target);
	bool next_upper_successor(const Point &pos, Point &next);
	int next_upper_successor_state;
};

