/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include "../headers/node.h"
#include "../headers/cost_map.h"
#include "../headers/dependency_map.h"
#include "../headers/path.h"
#include "../headers/heap.h"

#include "../headers/corners_non-blocking.h"

Corners_NonBlocking::Corners_NonBlocking(Cost_Map* base_map, Point& start, Point& goal, unsigned short nlevels, unsigned short xs, unsigned short ys) : Search(base_map, start, goal) {
	if (nlevels < 1)
		fprintf(stderr, "Invalid number of levels specified for SA*\n");
	levels = nlevels;
	x_rescale = xs;
	y_rescale = ys;
	reopen_margin = -1;
	fill_levels();
}

void
Corners_NonBlocking::fill_levels() {
	// Construct the cost maps
	cost_maps = (Cost_Map**) realloc(cost_maps, sizeof(Cost_Map*) * levels);
	unsigned short xs = 1;
	unsigned short ys = 1;
	for (int i = 1; i < levels; i++) {
		xs *= x_rescale;
		ys *= y_rescale;
		cost_maps[i] = new Cost_Map(cost_maps[i - 1], xs, ys);
		if (cost_maps[i]->uniform) {
///			printf("Enough maps at %d\n", i);
			levels = i+1;
			break;
		}
	}

	// Construct the path maps
	unsigned short full_width = cost_maps[0]->full_width;
	unsigned short full_height = cost_maps[0]->full_height;
	path_maps = (Path_Map**) realloc(path_maps, sizeof(Path_Map*) * levels);
	xs = 1;
	ys = 1;
	for (int i = 1; i < levels; i++) {
///		unsigned short width = cost_maps[i]->width;
///		unsigned short height = cost_maps[i]->height;
///		xs *= x_rescale;
///		ys *= y_rescale;
		// Currently we store the entire map at every level - very memory inefficient!
		// TODO: Shift to not storing interior vertices
		path_maps[i] = new Path_Map(i, full_width + 1, full_height + 1, (full_width + 1) * (full_height + 1));
	}

	// Construct the dependency maps
	dependencies = (Dependency_Map**) malloc(sizeof(Dependency_Map*) * (levels));
	dependencies[0] = new Dependency_Map(0, 0, 0);
	xs = 1;
	ys = 1;
	for (int i = 1; i < levels; i++) {
		xs *= x_rescale;
		ys *= y_rescale;
		unsigned short width = xs * (full_width / xs);
		if (width < full_width) width += xs;
		unsigned short height = ys * (full_height / xs);
		if (height < full_height) height += ys;
		dependencies[i] = new Dependency_Map(i, width + 1, height + 1);
	}
}

Corners_NonBlocking::~Corners_NonBlocking() {
	for (int i = 0; i < levels; i++)
		delete dependencies[i];
	free(dependencies);
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

At every level we are doing A* with the heuristic based on results from the
level above.  Any closed vertex can be used.  A vertex that hasn't been closed
is a dependecny, when it is closed the dependents are notified, and enqueued if
they are not waiting on anything else.  A single heap is used for all vertices
*/

bool
Corners_NonBlocking::next_successor(const Point &pos, Point &next) {
	int xs = cost_maps[pos.level]->x_scale;
	int ys = cost_maps[pos.level]->y_scale;
	// At the lowest level we always move 1 step in each direction
	if (pos.level == 0) {
		xs = 1;
		ys = 1;
	}
	if (pos.equals(next))
		next_successor_state = 0;
	switch (next_successor_state) {
		case 0:
			next_successor_state = 1;
			next.x = pos.x + xs;
			next.y = pos.y;
			return true;
		case 1:
			next_successor_state = 2;
			next.x = pos.x - xs;
			next.y = pos.y;
			return true;
		case 2:
			next_successor_state = 3;
			next.x = pos.x;
			next.y = pos.y + ys;
			return true;
		case 3:
			next_successor_state = 4;
			next.x = pos.x;
			next.y = pos.y - ys;
			return true;
		default:
			next_successor_state = 0;
			next.x = pos.x;
			next.y = pos.y;
			return false;
	}
}

double
Corners_NonBlocking::get_cost_of_interior(short xdiff, short ydiff, Node* cur, double cost, bool was_waiting) {
	Point next(cur->pos);
	next.level += 1;
	next.x -= xdiff;
	next.y -= ydiff;
///	next.print(stdout);
///	cur->pos.print(stdout);
	Node* node = get_node(next);
	if (! node->check_state(CLOSED)) {
		cur->set_state(WAITING);
		if (! was_waiting)
			dependencies[next.level]->add_dependency(cur, *node);
		return HUGE_VAL;
	} else {
		return node->g + cost * min(abs(xdiff), abs(ydiff));
	}
}

double
Corners_NonBlocking::get_cost_of_boundary(Point& next, Node* cur, double cost, bool was_waiting) {
	Node* node = get_node(next);
///	next.print(stdout);
///	cur->pos.print(stdout);
	if (! node->check_state(CLOSED)) {
		cur->set_state(WAITING);
		if (! was_waiting)
			dependencies[next.level]->add_dependency(cur, *node);
		return HUGE_VAL;
	} else {
		return node->g + cost;
	}
}

double
Corners_NonBlocking::heuristic(const Point &pos) {
	Point upper_pos(pos);
	upper_pos.level += 1;
	unsigned short upper_level = pos.level + 1;
	Point cur_goal(goal);
	cur_goal.level = upper_pos.level;
	if (pos.level % 2 == 1) {
		cur_goal.x = start.x;
		cur_goal.y = start.y;
	}

	// If we are out of levels return the manhattan distance
	if (upper_level >= levels)
		return manhattan_distance(cur_goal.x, cur_goal.y, pos.x, pos.y);


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
///	printf("dists = %u %u\n", x_dist, y_dist);
///	printf("scales = %u %u\n", xs, ys);
	Node* cur = get_node(pos);
	// Find answer for corner vertices
	if (x_dist == 0 && y_dist == 0) {
		Node* next = get_node(upper_pos);
		if (next->check_state(CLOSED)) {
			cur->unset_state(WAITING);
			return next->g;
		} else {
			dependencies[next->pos.level]->add_dependency(cur, *next);
			cur->set_state(WAITING);
			return HUGE_VAL;
		}
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
	bool was_waiting = cur->check_state(WAITING);
	cur->unset_state(WAITING);
	if (x_dist != 0 && y_dist != 0) {
		// Interior vertex
		Point step(upper_pos);
		step.x += 1;
		double cost = step_cost(upper_pos, step);

		double h  = get_cost_of_interior(x_dist, y_dist, cur, cost, was_waiting);
		if (h < best)
			best = h;

		h  = get_cost_of_interior(x_dist - xs, y_dist, cur, cost, was_waiting);
		if (h < best)
			best = h;

		h  = get_cost_of_interior(x_dist, y_dist - ys, cur, cost, was_waiting);
		if (h < best)
			best = h;

		h  = get_cost_of_interior(x_dist - xs, y_dist - ys, cur, cost, was_waiting);
		if (h < best)
			best = h;
	} else {
		// Boundary vertex
		// Do the two nearby
		Point next(upper_pos);
		next.x -= x_dist;
		next.y -= y_dist;
		double cost = step_cost(upper_pos, next);
		double h = get_cost_of_boundary(next, cur, cost, was_waiting);
		if (h < best)
			best = h;

		if (x_dist == 0)
			next.y += ys;
		else
			next.x += xs;
		cost = step_cost(upper_pos, next);
		h = get_cost_of_boundary(next, cur, cost, was_waiting);
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
			h = get_cost_of_boundary(next, cur, cost, was_waiting);
			if (h < best)
				best = h;
			if (x_dist == 0)
				next.y += ys;
			else
				next.x += xs;
			h = get_cost_of_boundary(next, cur, cost, was_waiting);
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
			if (cost > 0) {
				next.x -= x_dist;
				next.y -= y_dist;
				h = get_cost_of_boundary(next, cur, cost, was_waiting);
				if (h < best)
					best = h;
				if (x_dist == 0)
					next.y += ys;
				else
					next.x += xs;
				h = get_cost_of_boundary(next, cur, cost, was_waiting);
				if (h < best)
					best = h;
			}
		}
	}
///	printf("%f for ", best);
///	pos.print(stdout);
	return best;
}

void
Corners_NonBlocking::update(Point &next_pos, double g, Node* prev) {
	Node* next = get_node(next_pos);
	if (next->check_state(CLOSED)) {
		if (g < next->g) {
			double diff = next->g - g;
			if (diff > reopen_margin) {
				next->unset_state(CLOSED);
				next->g = g;
				next->f = next->g + next->h;
				next->prev = prev;
///				printf("Push 0   ");
///				next->print(stdout);
				fringe.push(next);
			}
		}
	} else if (next->check_state(FRINGE)) {
		if (g < next->g) {
			// update it
			next->g = g;
			next->f = next->g + next->h;
			next->prev = prev;
			fringe.update(next);
		}
	} else if (next->check_state(WAITING)) {
		if (g < next->g) {
			next->g = g;
			next->prev = prev;
			if (next->h != HUGE_VAL) {
				next->f = next->g + next->h;
				fringe.update(next);
			}
		}
	} else {
		next->g = g;
		next->prev = prev;
		next->h = heuristic(next_pos);
		if (next->h != HUGE_VAL) {
			next->f = next->g + next->h;
///			printf("Push 1   ");
///			next->print(stdout);
			fringe.push(next);
		}
	}
}

Node*
Corners_NonBlocking::pop() {
	Node* cur = fringe.pop();
///	printf("Pop        ");
///	cur->print(stdout);
	cur->set_state(CLOSED);
	expanded++;
	if (cur->g == HUGE_VAL)
		return cur;
	if (cur->pos.level > 0) {
		coarse_expanded++;
		vector<Node*>* waiting = dependencies[cur->pos.level]->get_dependencies(cur);
		if (waiting != NULL) {
			for (unsigned int i = 0; i < waiting->size(); i++) {
				Node* waiter = waiting->at(i);
				if (! waiter->check_state(CLOSED)) {
					waiter->h = heuristic(waiter->pos);
					if (waiter->check_state(FRINGE)) {
						waiter->f = waiter->g + waiter->h;
						fringe.update(waiter);
					} else if (waiter->h != HUGE_VAL) {
						waiter->f = waiter->g + waiter->h;
///						printf("Push 2   ");
///						waiter->print(stdout);
						fringe.push(waiter);
					}
				}
			}
			waiting->resize(0);
		}
	}

	return cur;
}

unsigned short
Corners_NonBlocking::min(unsigned short a, unsigned short b) {
	if (a < b)
		return a;
	else
		return b;
}

void
Corners_NonBlocking::make_interior_start(short xdiff, short ydiff, Point &cstart, double cost, Node* start_node) {
	Point next(cstart);
	next.x -= xdiff;
	next.y -= ydiff;
	Node* node = get_node(next);
	node->g = cost * min(abs(xdiff), abs(ydiff));
	node->h = heuristic(next);
	node->f = node->g + node->h;
	node->prev = start_node;
	node->set_state(START);
///	printf("Push 3   ");
///	node->print(stdout);
	fringe.push(node);
}

void
Corners_NonBlocking::make_boundary_start(Point& next, double cost, Node* start_node) {
	Node* node = get_node(next);
	node->g = cost;
	node->h = heuristic(next);
	node->f = node->g + node->h;
	node->prev = start_node;
	node->set_state(START);
///	printf("Push 4   ");
///	node->print(stdout);
	fringe.push(node);
}

Path
Corners_NonBlocking::search() {
	Node* goal_node = make_goal(goal);
	expanded = 0;

	Point cstart(start);
	Point cgoal(goal);
	for (int i = 0; i < levels; i++) {
		Node* node = get_node(cgoal);
		node->prev = NULL;
		node->set_state(GOAL);

		unsigned short search_level = cstart.level;
		Node* start_node = get_node(cstart);
		start_node->g = 0;
		start_node->h = heuristic(cstart);
		start_node->f = start_node->g + start_node->h;
		start_node->prev = NULL;
		start_node->set_state(START);
		if (search_level == 0) {
			if (! start_node->h != HUGE_VAL) {
///				printf("Push 5   ");
///				start_node->print(stdout);
				fringe.push(start_node);
			}
		} else {
			unsigned short xs = cost_maps[search_level]->x_scale;
			unsigned short ys = cost_maps[search_level]->y_scale;
			unsigned short x_dist = cstart.x % xs;
			unsigned short y_dist = cstart.y % ys;
			if (x_dist == 0 && y_dist == 0) {
				// Corner vertex
///				printf("Push 6   ");
///				node->print(stdout);
				fringe.push(start_node);
			} else if (x_dist != 0 && y_dist != 0) {
				// Interior vertex
				Point step(cstart);
				step.x += 1;
				double cost = step_cost(cstart, step);

				make_interior_start(x_dist, y_dist, cstart, cost, start_node);
				make_interior_start(x_dist - xs, y_dist, cstart, cost, start_node);
				make_interior_start(x_dist, y_dist - ys, cstart, cost, start_node);
				make_interior_start(x_dist - xs, y_dist - ys, cstart, cost, start_node);
			} else {
				// Boundary vertex
				// Do the two nearby
				Point next(cstart);
				next.x -= x_dist;
				next.y -= y_dist;
				double cost = step_cost(cstart, next);
				make_boundary_start(next, cost, start_node);

				if (x_dist == 0)
					next.y += ys;
				else
					next.x += xs;
				cost = step_cost(cstart, next);
				make_boundary_start(next, cost, start_node);

				// Do the four further away
				if (next.x == 0 || next.y == 0) {
				} else {
					next.x = cstart.x;
					next.y = cstart.y;
					if (x_dist == 0)
						next.x -= xs;
					else
						next.y -= ys;
					cost = step_cost(cstart, next);
					next.x -= x_dist;
					next.y -= y_dist;
					make_boundary_start(next, cost, start_node);
					if (x_dist == 0)
						next.y += ys;
					else
						next.x += xs;
					make_boundary_start(next, cost, start_node);
				}

				next.x = cstart.x;
				next.y = cstart.y;
				if (next.x == path_maps[next.level]->width || next.y == path_maps[next.level]->height) {
				} else {
					if (x_dist == 0)
						next.x += xs;
					else
						next.y += ys;
					cost = step_cost(cstart, next);
					if (cost > 0) {
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
		}

		cstart.level += 1;
		cgoal.level += 1;
		unsigned short tx = cstart.x;
		cstart.x = cgoal.x;
		cgoal.x = tx;
		unsigned short ty = cstart.y;
		cstart.y = cgoal.y;
		cgoal.y = ty;
	}

	Node* cur;
	Path ans(cost_maps[0]);
	while (! fringe.is_empty()) {
		cur = pop();
///		printf("Closed: ");
///		cur->print(stdout);

		if (cur == goal_node) {
			do {
				ans.add_node(Node(cur));
				cur = cur->prev;
			} while (cur != NULL);
			break;
		}

		if (cur->g == HUGE_VAL)
			break;

		Point next_pos(cur->pos);
		while (next_successor(cur->pos, next_pos)) {
			double cost = step_cost(cur->pos, next_pos);
			if (cost >= 0 && cost < HUGE_VAL) {
				double g = cur->g + cost;
				update(next_pos, g, cur);
			}
		}
	}


///	for (int i = 0; i < levels; i++) {
///		printf("level %d:\n", i);
///		printf("Map Costs\n");
///		cost_maps[i]->print(stdout);
///		printf("Path costs\n");
///		path_maps[i]->print(stdout);
///		printf("Waiting\n");
///		dependencies[i]->print(stdout);
///	}
///	ans.print(stdout);

	return ans;
}

