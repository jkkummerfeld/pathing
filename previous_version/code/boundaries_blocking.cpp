/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include "../headers/node.h"
#include "../headers/cost_map.h"
#include "../headers/path.h"
#include "../headers/heap.h"
#include "../headers/boundaries_blocking.h"

Boundaries_Blocking::Boundaries_Blocking(Cost_Map* base_map, Point& start, Point& goal, unsigned short nlevels, unsigned short xs, unsigned short ys) : Search(base_map, start, goal) {
	if (nlevels < 1)
		fprintf(stderr, "Invalid number of levels specified for SA*\n");
	levels = nlevels;
	x_rescale = xs;
	y_rescale = ys;
	fill_levels();
}

void
Boundaries_Blocking::fill_levels() {
	// Construct the cost map
	cost_maps = (Cost_Map**) realloc(cost_maps, sizeof(Cost_Map*) * levels);
	unsigned short xs = 1;
	unsigned short ys = 1;
	for (int i = 1; i < levels; i++) {
		xs *= x_rescale;
		ys *= y_rescale;
		cost_maps[i] = new Cost_Map(cost_maps[i - 1], xs, ys);
		if (cost_maps[i]->uniform) {
			levels = i+1;
			break;
		}
	}

	// Construct the path map
	path_maps = (Path_Map**) realloc(path_maps, sizeof(Path_Map*) * levels);
	xs = 1;
	ys = 1;
	unsigned short full_width = cost_maps[0]->full_width;
	unsigned short full_height = cost_maps[0]->full_height;
	for (int i = 1; i < levels; i++) {
///		unsigned short width = cost_maps[i]->width;
///		unsigned short height = cost_maps[i]->height;
		xs *= x_rescale;
		ys *= y_rescale;
		// We need the edges for each box (hence boxes * boundary vertices) and the bottom row and right most column
		path_maps[i] = new Path_Map(i, full_width, full_height, full_width * full_height);
		// path_maps[i] = new Path_Map(i, full_width, full_height, width * height * (x_rescale + y_rescale) + full_width + full_height);
		// printf("Creating path map with %d %d %d %d = %d * %d * (%d + %d) + %d + %d\n", i, full_width, full_height, width * height * (x_rescale + y_rescale) + full_width + full_height, width, height, x_rescale, y_rescale, full_width, full_height);
	}

	// Simple approach - one heap per level
	fringes = (Heap**) malloc(sizeof(Heap*) * levels);
	fringes[0] = &fringe;
	for (int i = 1; i < levels; i++)
		fringes[i] = new Heap(16384);
}

Boundaries_Blocking::~Boundaries_Blocking() {
	for (int i = 1; i < levels; i++)
		delete fringes[i];
	free(fringes);
}

/* Algorithm

At each level we are performing a search on some form of the map:

 - bottom, forward search, complete map
 - first level up, backward search, only the edges of a grid that is coarser than the map
 - second level up, forward search, now on an even coarser grid
 .
 .
 .
 - top level

Separate queues, whenever information is needed a query is sent up and the next
level returns the answer or continues searching until a value is found
*/

bool
Boundaries_Blocking::next_upper_successor(const Point &pos, Point &next) {
	int xs = cost_maps[pos.level]->x_scale;
	int ys = cost_maps[pos.level]->y_scale;
	if (pos.equals(next))
		next_upper_successor_state = 0;
	switch (next_upper_successor_state) {
		case 0:
			next_upper_successor_state = 1;
			if (pos.y % ys == 0)
				next.x = pos.x + 1;
			else if (pos.x % xs == 0)
				next.x = pos.x + xs;
			else
				next.x = pos.x + xs - (pos.x % xs);
			next.y = pos.y;
			return true;
		case 1:
			next_upper_successor_state = 2;
			if (pos.y % ys == 0)
				next.x = pos.x - 1;
			else if (pos.x % xs == 0)
				next.x = pos.x - xs;
			else
				next.x = pos.x - (pos.x % xs);
			next.y = pos.y;
			return true;
		case 2:
			next_upper_successor_state = 3;
			next.x = pos.x;
			if (pos.x % xs == 0)
				next.y = pos.y + 1;
			else if (pos.y % ys == 0)
				next.y = pos.y + ys;
			else
				next.y = pos.y + ys - (pos.y % ys);
			return true;
		case 3:
			next_upper_successor_state = 4;
			next.x = pos.x;
			if (pos.x % xs == 0)
				next.y = pos.y - 1;
			else if (pos.y % ys == 0)
				next.y = pos.y - ys;
			else
				next.y = pos.y - (pos.y % ys);
			return true;
		default:
			next_upper_successor_state = 0;
			next.x = pos.x;
			next.y = pos.y;
			return false;
	}
}

void
Boundaries_Blocking::coarse_search(Node* target) {
	int search_level = target->pos.level;
	// Start the search if it isn't in progress
	if (fringes[search_level]->is_empty()) {
		// printf("Starting search on level %d\n", search_level);
		Point upper_start(start);
		upper_start.level = target->pos.level;
		if (search_level % 2 == 1) {
			upper_start.x = goal.x;
			upper_start.y = goal.y;
		}
		Node* node = get_node(upper_start);
		if (node->check_state(CLOSED)) {
			return;
		}
		node->g = 0;
		// printf("Need heuristic for start on level %d\n", upper_start.level);
		node->h = heuristic(upper_start);
		node->f = node->g + node->h;
		node->prev = NULL;
		node->set_state(START);
		fringes[search_level]->push(node);
	}

	Node* cur;
	while (! fringes[search_level]->is_empty()) {
		cur = fringes[search_level]->pop();
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
						fringes[search_level]->update(next);
					}
				} else {
					next->g = g;
					next->h = heuristic(next_pos);
					next->f = next->g + next->h;
					next->prev = cur;
					fringes[search_level]->push(next);
				}
			}
		}

		// If the requested target is found, pause here
		if (cur == target)
			return;
	}
}

double
Boundaries_Blocking::heuristic(const Point &pos) {
	Point upper_pos(pos);
	upper_pos.level += 1;
	unsigned short upper_level = pos.level + 1;
	// printf("%d of %d\n", upper_pos.level, levels); fflush(stdout);
	// The goal alternates at each level
	Point cur_goal(goal);
	cur_goal.level = upper_pos.level;
	if (pos.level % 2 == 1) {
		cur_goal.x = start.x;
		cur_goal.y = start.y;
	}


	// If we are out of levels return the manhattan distance
	if (upper_level >= levels) {
		// printf("done\n");
		return manhattan_distance(cur_goal.x, cur_goal.y, pos.x, pos.y);
	}


	// If the map at this level is uniform return the true answer
	if (cost_maps[upper_level]->uniform) {
	  Point step(upper_pos);
	  if (cur_goal.x > pos.x)
	  	step.x += 1;
	  else if (cur_goal.x < pos.x)
	  	step.x -= 1;
	  else if (cur_goal.y > pos.y)
	  	step.y += 1;
	  else if (cur_goal.y < pos.y)
	  	step.y -= 1;
		double cost = step_cost(upper_pos, step);
		// printf("done\n");
		return cost * manhattan_distance(cur_goal.x, cur_goal.y, pos.x, pos.y);
	}


	unsigned short x_dist = pos.x % cost_maps[upper_level]->x_scale;
	unsigned short y_dist = pos.y % cost_maps[upper_level]->y_scale;
	// Find answer for boundary or corner vertices
	if (x_dist == 0 || y_dist == 0) {
		Node* next = get_node(upper_pos);
		// printf("Making query on level %d for pos on level %d\n", next->pos.level, upper_pos.level);
		if (! next->check_state(CLOSED))
			coarse_search(next);
		if (! next->check_state(CLOSED)) {
			// This position is unreachable
			return HUGE_VAL;
		}
		return next->g;
	}


	// Find answer for an interior point
	vector<Point*> coarse_locations;
	coarse_locations.push_back(new Point(pos.x - x_dist, pos.y, upper_level));
	coarse_locations.push_back(new Point(pos.x - x_dist + cost_maps[upper_level]->x_scale, pos.y, upper_level));
	coarse_locations.push_back(new Point(pos.x, pos.y - y_dist, upper_level));
	coarse_locations.push_back(new Point(pos.x, pos.y - y_dist + cost_maps[upper_level]->y_scale, upper_level));

	double best = HUGE_VAL;
	// if in the same block as the goal there is also the option to go directly to it
	if (((pos.x / cost_maps[upper_level]->x_scale) == (cur_goal.x / cost_maps[upper_level]->x_scale)) &&
	    ((pos.y / cost_maps[upper_level]->y_scale) == (cur_goal.y / cost_maps[upper_level]->y_scale))) {
	  Point step(upper_pos);
	  if (cur_goal.x > pos.x)
	  	step.x += 1;
	  else if (cur_goal.x < pos.x)
	  	step.x -= 1;
	  else if (cur_goal.y > pos.y)
	  	step.y += 1;
	  else if (cur_goal.y < pos.y)
	  	step.y -= 1;
		double cost = step_cost(upper_pos, step);
		best = cost * manhattan_distance(cur_goal.x, cur_goal.y, pos.x, pos.y);
	}
	    
	// Coarse search to find each of the values
	for (unsigned int i = 0; i < coarse_locations.size(); i++) {
		Node* next = get_node(*(coarse_locations[i]));
		// printf("Making query on level %d for pos on level %d\n", next->pos.level, upper_pos.level);
		if (! next->check_state(CLOSED))
			coarse_search(next);
		if (! next->check_state(CLOSED)) {
			// This position is unreachable
			delete coarse_locations[i];
			continue;
		}
		double cost = next->g + step_cost(next->pos, upper_pos);
		if (cost < best)
			best = cost;
		delete coarse_locations[i];
	}
	return best;
}

// TODO: Consider approaches to take into consideration the new perspective (e.g. coarse search returns the current f-value)
