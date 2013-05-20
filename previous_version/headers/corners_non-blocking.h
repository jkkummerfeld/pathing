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

class Corners_NonBlocking: public Search {
 public:
	// Data
	int coarse_expanded;

	// Methods
	Corners_NonBlocking(Cost_Map* base_map, Point& start, Point& goal, unsigned short nlevels, unsigned short xs, unsigned short ys);
	~Corners_NonBlocking();

	Path search();

	double reopen_margin;

 private:
	// Data
	Dependency_Map** dependencies;

	// Methods
	double heuristic(const Point &pos);
	void fill_levels();
	Node* pop();
	void update(Point &next_pos, double g, Node* prev);
	bool next_successor(const Point &pos, Point &next);

	unsigned short min(unsigned short a, unsigned short b);
	void make_boundary_start(Point& next, double cost, Node* start_node);
	void make_interior_start(short xdiff, short ydiff, Point &cstart, double cost, Node* start_node);
	double get_cost_of_interior(short xdiff, short ydiff, Node* cur, double cost, bool was_waiting);
	double get_cost_of_boundary(Point& next, Node* cur, double cost, bool was_waiting);
};

