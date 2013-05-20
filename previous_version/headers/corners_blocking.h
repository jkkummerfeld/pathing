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

class Corners_Blocking: public Search {
 public:
	// Data
	int coarse_expanded;

	// Methods
	Corners_Blocking(Cost_Map* base_map, Point& start, Point& goal, unsigned short nlevels, unsigned short xs, unsigned short ys);
	~Corners_Blocking();

	Path search();

	double reopen_margin;

 private:
	// Data
	Heap** fringes;

	// Methods
	double heuristic(const Point &pos);
	void fill_levels();
	void coarse_search(Node* target);
	bool next_upper_successor(const Point &pos, Point &next);
	int next_upper_successor_state;
	unsigned short min(unsigned short a, unsigned short b);
	void make_boundary_start(Point& next, double cost, Node* start_node);
	void make_interior_start(short xdiff, short ydiff, Point &upper_start, double cost, Node* start_node);
	double get_cost_of_interior(short xdiff, short ydiff, Point &upper_pos, double cost);
	double get_cost_of_boundary(Point& next, double cost);
};

