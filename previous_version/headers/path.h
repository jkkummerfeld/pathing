/*
 */

#pragma once

#include	<stdlib.h>
#include	<stdio.h>
#include <vector>

#include "node.h"
#include "cost_map.h"

using namespace std;

class Path {
 public:
	// Data
	vector<Node> nodes;
	double length;
	Cost_Map* cost_map;

	// Methods
	Path(Cost_Map* costs);
	~Path();

	void add_node(Node step);
	void print(FILE* out);

 private:
	// Data
	// Methods
};

