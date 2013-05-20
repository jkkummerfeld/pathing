/*
 */

#include	<cstdlib>
#include	<cstdio>
#include	<cmath>

#include "../headers/cost_map.h"
#include "../headers/heap.h"
#include "../headers/node.h"
#include "../headers/path_map.h"

#include "../headers/search.h"

Search::Search(Cost_Map* base, const Point &pstart, const Point &pgoal) :
	goal(pgoal),
	start(pstart),
	fringe(16384),
	next_successor_state(0) {
	// By default we have only a single level
	levels = 1;
	x_rescale = base->full_width;
	y_rescale = base->full_height;

	cost_maps = (Cost_Map**) malloc(sizeof(Cost_Map*));
	cost_maps[0] = base;

	unsigned short width = cost_maps[0]->full_width;
	unsigned short height = cost_maps[0]->full_height;
	path_maps = (Path_Map**) malloc(sizeof(Path_Map*));
	path_maps[0] = new Path_Map(0, width, height, width * height);
}

Search::~Search() {
	for (unsigned short l = 0; l < levels; l++) {
		if (l != 0) {
			delete cost_maps[l];
		}
		delete path_maps[l];
	}
	free(cost_maps);
	free(path_maps);
}

Node*
Search::get_node(const Point &p) {
	return path_maps[p.level]->get_node(p);
}

Node*
Search::make_goal(const Point &p) {
	Node* node = get_node(p);
	node->prev = NULL;
	node->set_state(GOAL);
	return node;
}

Node*
Search::make_start(const Point &p) {
	Node* node = get_node(p);
	node->g = 0;
	node->h = heuristic(p);
	node->f = node->h;
	node->prev = NULL;
	node->set_state(START);
///	printf("Push S   ");
///	node->print(stdout);
	return node;
}

bool
Search::next_successor(const Point &pos, Point &next) {
	if (pos.equals(next))
		next_successor_state = 0;
	switch (next_successor_state) {
		case 0:
			next_successor_state = 1;
			next.x = pos.x + 1;
			next.y = pos.y;
			return true;
		case 1:
			next_successor_state = 2;
			next.x = pos.x - 1;
			next.y = pos.y;
			return true;
		case 2:
			next_successor_state = 3;
			next.x = pos.x;
			next.y = pos.y + 1;
			return true;
		case 3:
			next_successor_state = 4;
			next.x = pos.x;
			next.y = pos.y - 1;
			return true;
		default:
			next_successor_state = 0;
			return false;
	}
}

double
Search::step_cost(const Point& from, const Point& to) {
	return cost_maps[from.level]->edge_cost(from, to);
}

Path
Search::search() {
	expanded = 0;
	Node* cur;
	Path ans(cost_maps[0]);
	make_goal(goal);
	fringe.push(make_start(start));
	double cur_f = -1;
	int iter = 0;
	while (! fringe.is_empty()) {
		iter++;
		cur = fringe.pop();
		cur->set_state(CLOSED);
///		printf("Pop      ");
///		cur->print(stdout);
		if (cur_f - cur->f > 1e-6 ) {
			fprintf(stderr, "f value of node being expanded decreased\n");
			fprintf(stdout, "Node above has f-value lower than previously closed node (%.1lf < %.1lf)\n", cur->f, cur_f);
		}
		cur_f = cur->f;
		expanded++;

		// If we are done, construct the path
		if (cur->check_state(GOAL)) {
			do {
				ans.add_node(Node(cur));
				cur = cur->prev;
			} while (cur != NULL);
			break;
		}

		// Consider neighbours
		Point next_pos(cur->pos);
		while (next_successor(cur->pos, next_pos)) {
			double cost = step_cost(cur->pos, next_pos);
			if (cost >= 0 && cost < HUGE_VAL) {
				double ncost = cur->g + cost;
				Node* next = get_node(next_pos);
				if (next->check_state(CLOSED)) {
					continue;
				} else if (next->check_state(FRINGE)) {
					if (ncost < next->g) {
						// update it
						next->g = ncost;
						next->f = next->g + next->h;
						next->prev = cur;
						fringe.update(next);
					}
				} else {
					next->h = heuristic(next_pos);
					next->g = ncost;
					next->f = next->g + next->h;
					next->prev = cur;
///					printf("Push 0   ");
///					next->print(stdout);
					fringe.push(next);
				}
			}
		}
	}

///	for (int i = 0; i < levels; i++) {
///		printf("level %d:\n", i);
///		printf("Map Costs\n");
///		cost_maps[i]->print(stdout);
///		printf("Path costs\n");
///		path_maps[i]->print(stdout);
///	}
///	ans.print(stdout);

	return ans;
}

double
euclidean_distance(short x1, short y1, short x2, short y2) {
	return sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));
}

double
manhattan_distance(short x1, short y1, short x2, short y2) {
	return abs(x1 - x2) + abs(y1 - y2);
}

double
octile_distance(short x1, short y1, short x2, short y2) {
	short dx = abs(x2 - x1);
	short dy = abs(y2 - y1);
	return sqrt(2.0) * min(dx, dy) + abs(dx - dy);
}

