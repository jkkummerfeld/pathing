/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include "../headers/node.h"
#include "../headers/cost_map.h"
#include "../headers/path_map.h"
#include "../headers/path.h"
#include "../headers/heap.h"
#include "../headers/cucs_heuristic.h"

CUCS_Heuristic::CUCS_Heuristic(Cost_Map* base_map, Point& start, Point& goal) : Search(base_map, start, goal), upper_fringe(16384), upper_goal(start), upper_start(goal) {
	upper_goal.level += 1;
	upper_start.level += 1;
	levels = 2;
	x_rescale = 4;
	y_rescale = 4;
	// Construct the cost map
	cost_maps = (Cost_Map**) realloc(cost_maps, sizeof(Cost_Map*) * levels);
	cost_maps[1] = new Cost_Map(cost_maps[0], x_rescale, y_rescale);

	// Construct the path map
	path_maps = (Path_Map**) realloc(path_maps, sizeof(Path_Map*) * levels);
	unsigned short width = cost_maps[0]->full_width;
	unsigned short height = cost_maps[0]->full_height;
	path_maps[1] = new Path_Map(1, width, height, width * height); // this is far more than needed, TODO: use a perfect hash
}

CUCS_Heuristic::~CUCS_Heuristic() {
}

bool
CUCS_Heuristic::next_upper_successor(const Point &pos, Point &next) {
	if (pos.equals(next))
		next_upper_successor_state = 0;
	switch (next_upper_successor_state) {
		case 0:
			next_upper_successor_state = 1;
			if (pos.y % y_rescale == 0)
				next.x = pos.x + 1;
			else if (pos.x % x_rescale == 0)
				next.x = pos.x + x_rescale;
			else
				next.x = pos.x + x_rescale - (pos.x % x_rescale);
			next.y = pos.y;
			return true;
		case 1:
			next_upper_successor_state = 2;
			if (pos.y % y_rescale == 0)
				next.x = pos.x - 1;
			else if (pos.x % x_rescale == 0)
				next.x = pos.x - x_rescale;
			else
				next.x = pos.x - (pos.x % x_rescale);
			next.y = pos.y;
			return true;
		case 2:
			next_upper_successor_state = 3;
			next.x = pos.x;
			if (pos.x % x_rescale == 0)
				next.y = pos.y + 1;
			else if (pos.y % y_rescale == 0)
				next.y = pos.y + y_rescale;
			else
				next.y = pos.y + y_rescale - (pos.y % y_rescale);
			return true;
		case 3:
			next_upper_successor_state = 4;
			next.x = pos.x;
			if (pos.x % x_rescale == 0)
				next.y = pos.y - 1;
			else if (pos.y % y_rescale == 0)
				next.y = pos.y - y_rescale;
			else
				next.y = pos.y - (pos.y % y_rescale);
			return true;
		default:
			next_upper_successor_state = 0;
			next.x = pos.x;
			next.y = pos.y;
			return false;
	}
}

void
CUCS_Heuristic::coarse_search(Node* target) {
	// printf("Searching for: ");
	// target->print(stdout);

	// Start the search if it isn't in progress
	if (upper_fringe.is_empty()) {
		Node* node = get_node(upper_start);
		node->g = 0;
		node->h = 0;
		node->f = node->g + node->h;
		node->prev = NULL;
		node->set_state(START);
		upper_fringe.push(node);
	}

	Node* cur;
	while (! upper_fringe.is_empty()) {
		cur = upper_fringe.pop();
		cur->set_state(CLOSED);
		expanded++;
		coarse_expanded++;

		// Consider neighbours
		Point next_pos(cur->pos);
		while (next_upper_successor(cur->pos, next_pos)) {
			double cost = step_cost(cur->pos, next_pos);
			if (cost >= 0 && cost < HUGE_VAL) {
				double g = cur->g + cost;
				Node* next = get_node(next_pos);
				if (next->check_state(CLOSED)) {
					continue;
				} else if (next->check_state(FRINGE)) {
					if (g < next->g) {
						// update it
						next->g = g;
						next->f = next->g + next->h;
						next->prev = cur;
						upper_fringe.update(next);
					}
				} else {
					next->g = g;
					next->h = 0;
					next->f = next->g + next->h;
					next->prev = cur;
					upper_fringe.push(next);
				}
			}
		}

		// If the requested target is found, pause here
		if (cur == target)
			return;
	}
}

double
CUCS_Heuristic::heuristic(const Point &pos) {
	Point upper_pos(pos);
	upper_pos.level = 1;
	// Determine corresponding point[s] in coarse grid
	vector<Point*> coarse_locations;
	unsigned short x_dist = upper_pos.x % x_rescale;
	unsigned short y_dist = upper_pos.y % y_rescale;

	if (x_dist == 0 || y_dist == 0) {
		Node* next = get_node(upper_pos);
		if (! next->check_state(CLOSED))
			coarse_search(next);
		return next->g;
	} else {
		coarse_locations.push_back(new Point(upper_pos.x - x_dist, upper_pos.y, 1));
		coarse_locations.push_back(new Point(upper_pos.x - x_dist + x_rescale, upper_pos.y, 1));
		coarse_locations.push_back(new Point(upper_pos.x, upper_pos.y - y_dist, 1));
		coarse_locations.push_back(new Point(upper_pos.x, upper_pos.y - y_dist + y_rescale, 1));
	}

	double best = HUGE_VAL;
	// if in the same block as the goal there is also the option to go directly to it
	if ((upper_pos.x / x_rescale == goal.x / x_rescale) &&
	    (upper_pos.y / y_rescale == goal.y / y_rescale)) {
	  Point step(upper_pos);
	  if (goal.x > pos.x)
	  	step.x += 1;
	  else if (goal.x < pos.x)
	  	step.x -= 1;
	  else if (goal.y > pos.y)
	  	step.y += 1;
	  else if (goal.y < pos.y)
	  	step.y -= 1;
		double cost = step_cost(upper_pos, step);
		best = cost * manhattan_distance(goal.x, goal.y, pos.x, pos.y);
	}
	    
	// Coarse search to find each of the values
	for (unsigned int i = 0; i < coarse_locations.size(); i++) {
		Node* next = get_node(*(coarse_locations[i]));
		if (! next->check_state(CLOSED))
			coarse_search(next);
		double dist = next->g + step_cost(next->pos, upper_pos);
		if (dist < best)
			best = dist;
		delete coarse_locations[i];
	}
	return best;
}

