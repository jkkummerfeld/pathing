/*
 */

#include	<stdlib.h>
#include	<stdio.h>

#include "../headers/path.h"
#include "../headers/node.h"
#include "../headers/cost_map.h"

Path::Path(Cost_Map* costs) {
	length = -1;
	cost_map = costs;
}

Path::~Path() {
}

void
Path::add_node(Node step) {
	if (nodes.size() > 0)
		length += cost_map->edge_cost(step.pos, nodes.back().pos);
	else
		length = 0;
	nodes.push_back(step);
}

void
Path::print(FILE* out) {
	if (nodes.size() == 0)
		fprintf(out, "Empty path\n");
	else
		fprintf(out, "Path of cost %lf\n", length);
	for (unsigned int i = 0; i < nodes.size(); i++) {
		nodes[i].print(out);
	}
}

