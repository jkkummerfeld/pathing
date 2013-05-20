/*
 */

#pragma once

#include "node.h"
#include "path.h"
#include "cost_map.h"
#include "path_map.h"
#include "heap.h"

class Search {
 public:
	// Data
	int expanded;

	// Methods
	Search(Cost_Map* base_map, const Point &start, const Point &goal);
	~Search();

	Path search();

 protected:
	// Data
	Cost_Map** cost_maps;
	Path_Map** path_maps;
	unsigned short levels;
	unsigned short x_rescale;
	unsigned short y_rescale;
	Point goal;
	Point start;
	Heap fringe;

	// Methods and state data
	Node* get_node(const Point &p);
	Node* make_goal(const Point &p);
	Node* make_start(const Point &p);
	bool next_successor(const Point &pos, Point &next);
	int next_successor_state;
	double step_cost(const Point &from, const Point &to);

	// Virtual methods
	virtual double heuristic(const Point &pos) = 0;
};


double euclidean_distance(short x1, short y1, short x2, short y2);
double manhattan_distance(short x1, short y1, short x2, short y2);
double octile_distance(short x1, short y1, short x2, short y2);

