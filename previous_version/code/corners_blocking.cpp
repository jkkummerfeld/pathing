/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include "../headers/node.h"
#include "../headers/cost_map.h"
#include "../headers/path.h"
#include "../headers/heap.h"
#include "../headers/corners_blocking.h"

Corners_Blocking::Corners_Blocking(Cost_Map* base_map, Point& start, Point& goal, unsigned short nlevels, unsigned short xs, unsigned short ys) : Search(base_map, start, goal) {
	if (nlevels < 1)
		fprintf(stderr, "Invalid number of levels specified for SA*\n");
	levels = nlevels;
	x_rescale = xs;
	y_rescale = ys;
	reopen_margin = -1;
	fill_levels();
}

void
Corners_Blocking::fill_levels() {
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

Corners_Blocking::~Corners_Blocking() {
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
Corners_Blocking::next_upper_successor(const Point &pos, Point &next) {
	int xs = cost_maps[pos.level]->x_scale;
	int ys = cost_maps[pos.level]->y_scale;
	if (pos.equals(next))
		next_upper_successor_state = 0;
	switch (next_upper_successor_state) {
		case 0:
			next_upper_successor_state = 1;
			next.x = pos.x + xs;
			next.y = pos.y;
			return true;
		case 1:
			next_upper_successor_state = 2;
			next.x = pos.x - xs;
			next.y = pos.y;
			return true;
		case 2:
			next_upper_successor_state = 3;
			next.x = pos.x;
			next.y = pos.y + ys;
			return true;
		case 3:
			next_upper_successor_state = 4;
			next.x = pos.x;
			next.y = pos.y - ys;
			return true;
		default:
			next_upper_successor_state = 0;
			next.x = pos.x;
			next.y = pos.y;
			return false;
	}
}

unsigned short
Corners_Blocking::min(unsigned short a, unsigned short b) {
	if (a < b)
		return a;
	else
		return b;
}

void
Corners_Blocking::make_interior_start(short xdiff, short ydiff, Point &upper_start, double cost, Node* start_node) {
	Point next(upper_start);
	next.x -= xdiff;
	next.y -= ydiff;
	Node* node = get_node(next);
	node->g = cost * min(abs(xdiff), abs(ydiff));
	node->h = heuristic(next);
	node->f = node->g + node->h;
	node->prev = start_node;
	node->set_state(START);
///	printf("Push 2   ");
///	node->print(stdout);
	fringes[next.level]->push(node);
}

void
Corners_Blocking::make_boundary_start(Point& next, double cost, Node* start_node) {
	Node* node = get_node(next);
	node->g = cost;
	node->h = heuristic(next);
	node->f = node->g + node->h;
	node->prev = start_node;
	node->set_state(START);
///	printf("Push 3   ");
///	node->print(stdout);
	fringes[next.level]->push(node);
}

void
Corners_Blocking::coarse_search(Node* target) {
	int search_level = target->pos.level;
	// Start the search if it isn't in progress
	if (fringes[search_level]->is_empty()) {
		Point upper_start(start);
		upper_start.level = target->pos.level;
		if (search_level % 2 == 1) {
			upper_start.x = goal.x;
			upper_start.y = goal.y;
		}
		unsigned short xs = cost_maps[upper_start.level]->x_scale;
		unsigned short ys = cost_maps[upper_start.level]->y_scale;
		unsigned short x_dist = upper_start.x % xs;
		unsigned short y_dist = upper_start.y % ys;
		Node* start_node = get_node(upper_start);
		// Check if we've actually completely done the search
		if (start_node->g == 0)
			return;
		start_node->g = 0;
		start_node->h = heuristic(upper_start);
		start_node->f = start_node->g + start_node->h;
		start_node->prev = NULL;
		start_node->set_state(START);
		if (x_dist == 0 && y_dist == 0) {
			// Corner vertex
///			printf("Push 4   ");
///			start_node->print(stdout);
			fringes[search_level]->push(start_node);
		} else if (x_dist != 0 && y_dist != 0) {
			// Interior vertex
			Point step(upper_start);
			step.x += 1;
			double cost = step_cost(upper_start, step);

			make_interior_start(x_dist, y_dist, upper_start, cost, start_node);
			make_interior_start(x_dist - xs, y_dist, upper_start, cost, start_node);
			make_interior_start(x_dist, y_dist - ys, upper_start, cost, start_node);
			make_interior_start(x_dist - xs, y_dist - ys, upper_start, cost, start_node);
		} else {
			// Boundary vertex
			// Do the two nearby
			Point next(upper_start);
			next.x -= x_dist;
			next.y -= y_dist;
			double cost = step_cost(upper_start, next);
			make_boundary_start(next, cost, start_node);

			if (x_dist == 0)
				next.y += ys;
			else
				next.x += xs;
			cost = step_cost(upper_start, next);
			make_boundary_start(next, cost, start_node);

			// Do the four further away
			if (next.x == 0 || next.y == 0) {
			} else {
				next.x = upper_start.x;
				next.y = upper_start.y;
				if (x_dist == 0)
					next.x -= xs;
				else
					next.y -= ys;
				cost = step_cost(upper_start, next);
				next.x -= x_dist;
				next.y -= y_dist;
				make_boundary_start(next, cost, start_node);
				if (x_dist == 0)
					next.y += ys;
				else
					next.x += xs;
				make_boundary_start(next, cost, start_node);
			}

			next.x = upper_start.x;
			next.y = upper_start.y;
			if (next.x == path_maps[next.level]->width || next.y == path_maps[next.level]->height) {
			} else {
				if (x_dist == 0)
					next.x += xs;
				else
					next.y += ys;
				cost = step_cost(upper_start, next);
				if (cost < 0)
					cost = HUGE_VAL;
				next.x -= x_dist;
				next.y -= y_dist;
				make_boundary_start(next, cost, start_node);
				if (x_dist == 0)
					next.y += ys;
				else
					next.x += xs;
				make_boundary_start(next, cost, start_node);
			}
		}
	}

	Node* cur;
	while (! fringes[search_level]->is_empty()) {
		cur = fringes[search_level]->pop();
		cur->set_state(CLOSED);
///		printf("Pop      ");
///		cur->print(stdout);
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
					if (g < next->g) {
						double diff = next->g - g;
						if (diff > reopen_margin) {
						// re-push it
							next->unset_state(CLOSED);
							next->g = g;
							next->f = next->g + next->h;
							next->prev = cur;
///							printf("Push 6   ");
///							next->print(stdout);
							fringes[search_level]->push(next);
						}
					}
				} else if (next->check_state(FRINGE)) {
					if (g < next->g) {
						// update it
						next->g = g;
						next->f = next->g + next->h;
						next->prev = cur;
///						printf("Update 0 ");
///						next->print(stdout);
						fringes[search_level]->update(next);
					}
				} else {
					next->g = g;
					next->h = heuristic(next_pos);
					next->f = next->g + next->h;
					next->prev = cur;
///					printf("Push 1   ");
///					next->print(stdout);
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
Corners_Blocking::get_cost_of_interior(short xdiff, short ydiff, Point &upper_pos, double cost) {
	Point next(upper_pos);
	next.x -= xdiff;
	next.y -= ydiff;
	Node* node = get_node(next);
	if (! node->check_state(CLOSED))
		coarse_search(node);
	if (! node->check_state(CLOSED)) {
		// This position is unreachable
		return HUGE_VAL;
	}
	return node->g + cost * min(abs(xdiff), abs(ydiff));
}

double
Corners_Blocking::get_cost_of_boundary(Point& next, double cost) {
	Node* node = get_node(next);
	if (! node->check_state(CLOSED))
		coarse_search(node);
	if (! node->check_state(CLOSED)) {
		// This position is unreachable
		return HUGE_VAL;
	}
	return node->g + cost;
}

double
Corners_Blocking::heuristic(const Point &pos) {
	Point upper_pos(pos);
	upper_pos.level += 1;
	unsigned short upper_level = pos.level + 1;
	// The goal alternates at each level
	Point cur_goal(goal);
	cur_goal.level = upper_pos.level;
	if (pos.level % 2 == 1) {
		cur_goal.x = start.x;
		cur_goal.y = start.y;
	}


	// If we are out of levels return the manhattan distance
	if (upper_level >= levels) {
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
		return cost * manhattan_distance(cur_goal.x, cur_goal.y, pos.x, pos.y);
	}


	unsigned short xs = cost_maps[upper_pos.level]->x_scale;
	unsigned short ys = cost_maps[upper_pos.level]->y_scale;
	unsigned short x_dist = pos.x % xs;
	unsigned short y_dist = pos.y % ys;
	// Find answer for corner vertices
	if (x_dist == 0 && y_dist == 0) {
		Node* next = get_node(upper_pos);
		if (! next->check_state(CLOSED))
			coarse_search(next);
		if (! next->check_state(CLOSED)) {
			// This position is unreachable
			return HUGE_VAL;
		}
		return next->g;
	}


	// Find answer for a non-corner point
	double best = HUGE_VAL;
	// if in the same block as the goal there is also the option to go directly
	// to it
	if ((((pos.x / xs) == (cur_goal.x / xs)) &&
	     ((pos.y / ys) == (cur_goal.y / ys))) ||
	    ((cur_goal.x % xs == 0) &&
	     ((pos.x / xs) == -1 + (cur_goal.x / xs)) &&
	     ((pos.y / ys) == (cur_goal.y / ys))) ||
	    ((cur_goal.y % ys == 0) &&
	     ((pos.x / xs) == (cur_goal.x / xs)) &&
	     ((pos.y / ys) == -1 + (cur_goal.y / ys))) ||
	    ((x_dist == 0) &&
	     ((pos.x / xs) == 1 + (cur_goal.x / xs)) &&
	     ((pos.y / ys) == (cur_goal.y / ys))) ||
	    ((y_dist == 0) &&
	     ((pos.x / xs) == (cur_goal.x / xs)) &&
	     ((pos.y / ys) == 1 + (cur_goal.y / ys)))
	   ) {
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
	if (x_dist != 0 && y_dist != 0) {
		// Interior vertex
		Point step(upper_pos);
		step.x += 1;
		double cost = step_cost(upper_pos, step);

		double h  = get_cost_of_interior(x_dist, y_dist, upper_pos, cost);
		if (h < best)
			best = h;

		h  = get_cost_of_interior(x_dist - xs, y_dist, upper_pos, cost);
		if (h < best)
			best = h;

		h  = get_cost_of_interior(x_dist, y_dist - ys, upper_pos, cost);
		if (h < best)
			best = h;

		h  = get_cost_of_interior(x_dist - xs, y_dist - ys, upper_pos, cost);
		if (h < best)
			best = h;
	} else {
		// Boundary vertex
		// Do the two nearby
		Point next(upper_pos);
		next.x -= x_dist;
		next.y -= y_dist;
		double cost = step_cost(upper_pos, next);
		double h = get_cost_of_boundary(next, cost);
		if (h < best)
			best = h;

		if (x_dist == 0)
			next.y += ys;
		else
			next.x += xs;
		cost = step_cost(upper_pos, next);
		h = get_cost_of_boundary(next, cost);
		if (h < best)
			best = h;

		// Do the four further away
		if (next.x == 0 || next.y == 0) {
		} else {
			next.x = upper_pos.x;
			next.y = upper_pos.y;
			if (x_dist == 0)
				next.x -= xs;
			else
				next.y -= ys;
			cost = step_cost(upper_pos, next);
			next.x -= x_dist;
			next.y -= y_dist;
			h = get_cost_of_boundary(next, cost);
			if (h < best)
				best = h;
			if (x_dist == 0)
				next.y += ys;
			else
				next.x += xs;
			h = get_cost_of_boundary(next, cost);
			if (h < best)
				best = h;
		}

		next.x = upper_pos.x;
		next.y = upper_pos.y;
		if (next.x == path_maps[next.level]->width || next.y == path_maps[next.level]->height) {
		} else {
			if (x_dist == 0)
				next.x += xs;
			else
				next.y += ys;
			cost = step_cost(upper_pos, next);
			if (cost < 0)
				cost = HUGE_VAL;
			next.x -= x_dist;
			next.y -= y_dist;
			h = get_cost_of_boundary(next, cost);
			if (h < best)
				best = h;
			if (x_dist == 0)
				next.y += ys;
			else
				next.x += xs;
			h = get_cost_of_boundary(next, cost);
			if (h < best)
				best = h;
		}
	}
	return best;
}

Path
Corners_Blocking::search() {
	expanded = 0;
	Node* cur;
	Path ans(cost_maps[0]);
	make_goal(goal);
	fringe.push(make_start(start));
	while (! fringe.is_empty()) {
		cur = fringe.pop();
		cur->set_state(CLOSED);
///		printf("Pop      ");
///		cur->print(stdout);
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
					if (ncost < next->g) {
						// update it
						next->unset_state(CLOSED);
						next->g = ncost;
						next->f = next->g + next->h;
						next->prev = cur;
///						printf("Push 5   ");
///						next->print(stdout);
						fringe.push(next);
					}
				} else if (next->check_state(FRINGE)) {
					if (ncost < next->g) {
						// update it
						next->g = ncost;
						next->f = next->g + next->h;
						next->prev = cur;
///						printf("Update 1 ");
///						next->print(stdout);
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

// TODO: Consider approaches to take into consideration the new perspective (e.g. coarse search returns the current f-value)
