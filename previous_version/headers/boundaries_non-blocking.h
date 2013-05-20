/*
 */

#pragma once

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include "node.h"
#include "path.h"
#include "cost_map.h"
#include "dependency_map.h"
#include "heap.h"
#include "search.h"

class Boundaries_NonBlocking: public Search {
 public:
	// Data
	int coarse_expanded;

	// Methods
	Boundaries_NonBlocking(Cost_Map* base_map, Point& start, Point& goal, unsigned short nlevels, unsigned short xs, unsigned short ys);
	~Boundaries_NonBlocking();

	Path search();

 private:
	// Data
	Dependency_Map** dependencies;

	// Methods
	double heuristic(const Point &pos);
	void fill_levels();
	Node* pop();
	void update(Point &next_pos, double g, Node* prev);
	bool next_successor(const Point &pos, Point &next);
};

