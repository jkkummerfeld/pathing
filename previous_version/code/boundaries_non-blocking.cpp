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

#include "../headers/boundaries_non-blocking.h"

Boundaries_NonBlocking::Boundaries_NonBlocking(Cost_Map* base_map, Point& start, Point& goal, unsigned short nlevels, unsigned short xs, unsigned short ys) : Search(base_map, start, goal) {
	if (nlevels < 1)
		fprintf(stderr, "Invalid number of levels specified for SA*\n");
	levels = nlevels;
	x_rescale = xs;
	y_rescale = ys;
	fill_levels();
}

void
Boundaries_NonBlocking::fill_levels() {
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

Boundaries_NonBlocking::~Boundaries_NonBlocking() {
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
Boundaries_NonBlocking::next_successor(const Point &pos, Point &next) {
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
			if (pos.y % ys == 0)
				next.x = pos.x + 1;
			else if (pos.x % xs == 0)
				next.x = pos.x + xs;
			else
				next.x = pos.x + xs - (pos.x % xs);
			next.y = pos.y;
			return true;
		case 1:
			next_successor_state = 2;
			if (pos.y % ys == 0)
				next.x = pos.x - 1;
			else if (pos.x % xs == 0)
				next.x = pos.x - xs;
			else
				next.x = pos.x - (pos.x % xs);
			next.y = pos.y;
			return true;
		case 2:
			next_successor_state = 3;
			next.x = pos.x;
			if (pos.x % xs == 0)
				next.y = pos.y + 1;
			else if (pos.y % ys == 0)
				next.y = pos.y + ys;
			else
				next.y = pos.y + ys - (pos.y % ys);
			return true;
		case 3:
			next_successor_state = 4;
			next.x = pos.x;
			if (pos.x % xs == 0)
				next.y = pos.y - 1;
			else if (pos.y % ys == 0)
				next.y = pos.y - ys;
			else
				next.y = pos.y - (pos.y % ys);
			return true;
		default:
			next_successor_state = 0;
			next.x = pos.x;
			next.y = pos.y;
			return false;
	}
}

double
Boundaries_NonBlocking::heuristic(const Point &pos) {
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


	unsigned short x_dist = pos.x % cost_maps[upper_level]->x_scale;
	unsigned short y_dist = pos.y % cost_maps[upper_level]->y_scale;
	Node* cur = get_node(pos);
	// Find answer for boundary or corner vertices
	if (x_dist == 0 || y_dist == 0) {
		Node* next = get_node(upper_pos);
		if (next->check_state(CLOSED)) {
			cur->unset_state(WAITING);
			return next->g;
		} else {
			Node* cur = get_node(pos);
			dependencies[next->pos.level]->add_dependency(cur, *next);
			cur->set_state(WAITING);
			return HUGE_VAL;
		}
	}


	// Find answer for an interior point
	vector<Point*> coarse_locations;
	coarse_locations.push_back(new Point(pos.x - x_dist, pos.y, upper_level));
	coarse_locations.push_back(new Point(pos.x - x_dist + cost_maps[upper_level]->x_scale, pos.y, upper_level));
	coarse_locations.push_back(new Point(pos.x, pos.y - y_dist, upper_level));
	coarse_locations.push_back(new Point(pos.x, pos.y - y_dist + cost_maps[upper_level]->y_scale, upper_level));

	double best = HUGE_VAL;
	// if in the same block as the goal there is also the option to go directly to it
	if ((pos.x / cost_maps[upper_level]->x_scale == cur_goal.x / cost_maps[upper_level]->x_scale) &&
	    (pos.y / cost_maps[upper_level]->y_scale == cur_goal.y / cost_maps[upper_level]->y_scale)) {
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
	    
	// Check for the values it dependes on
	bool was_waiting = cur->check_state(WAITING);
	cur->unset_state(WAITING);
	for (unsigned int i = 0; i < coarse_locations.size(); i++) {
		Node* next = get_node(*(coarse_locations[i]));
		if (! next->check_state(CLOSED)) {
			cur->set_state(WAITING);
			if (! was_waiting) {
				dependencies[next->pos.level]->add_dependency(cur, *next);
			}
		} else {
			double cost = next->g + step_cost(next->pos, upper_pos);
			if (cost < best)
				best = cost;
		}
		delete coarse_locations[i];
	}
///	printf("%f for ", best);
///	pos.print(stdout);
	return best;
}

void
Boundaries_NonBlocking::update(Point &next_pos, double g, Node* prev) {
	Node* next = get_node(next_pos);
	if (next->check_state(CLOSED)) {
		// this heuristic is consistent, so we will never have to re-open vertices
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
///			printf("1 Push     ");
///			next->print(stdout);
			fringe.push(next);
		}
	}
}

Node*
Boundaries_NonBlocking::pop() {
	Node* cur = fringe.pop();
///	printf("Pop        ");
///	cur->print(stdout);
	cur->set_state(CLOSED);
	expanded++;
	if (cur->pos.level > 0) {
		coarse_expanded++;
		vector<Node*>* waiting = dependencies[cur->pos.level]->get_dependencies(cur);
		if (waiting != NULL) {
			for (unsigned int i = 0; i < waiting->size(); i++) {
				if (! waiting->at(i)->check_state(CLOSED)) {
					waiting->at(i)->h = heuristic(waiting->at(i)->pos);
					if (waiting->at(i)->check_state(FRINGE)) {
						waiting->at(i)->f = waiting->at(i)->g + waiting->at(i)->h;
						fringe.update(waiting->at(i));
					} else if (waiting->at(i)->h != HUGE_VAL) {
						waiting->at(i)->f = waiting->at(i)->g + waiting->at(i)->h;
///						printf("2 Push     ");
///						waiting->at(i)->print(stdout);
						fringe.push(waiting->at(i));
					}
				}
			}
			waiting->resize(0);
		}
	}

	return cur;
}

Path
Boundaries_NonBlocking::search() {
	Node* goal_node = make_goal(goal);
	expanded = 0;

	Point cstart(start);
	Point cgoal(goal);
	for (int i = 0; i < levels; i++) {
		Node* node = get_node(cgoal);
		node->prev = NULL;
		node->set_state(GOAL);
///		printf("Created goal:\n");
///		node->print(stdout);

		node = get_node(cstart);
		node->g = 0;
		node->h = heuristic(cstart);
		node->f = node->h;
		node->prev = NULL;
		node->set_state(START);
/// 		printf("Created start:\n");
///		node->print(stdout);
		if (! node->h != HUGE_VAL) {
///			printf("4 Pushed\n");
			fringe.push(node);
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
	double cur_f = -1;
	while (! fringe.is_empty()) {
		cur = pop();
		if (cur_f - cur->f > 1e-6 ) {
			fprintf(stderr, "f value of node being expanded decreased\n");
			fprintf(stdout, "Node above has f-value lower than previously closed node (%.1lf < %.1lf)\n", cur->f, cur_f);
		}
		cur_f = cur->f;
///		printf("Closed: ");
///		cur->print(stdout);

		if (cur == goal_node) {
			do {
				ans.add_node(Node(cur));
				cur = cur->prev;
			} while (cur != NULL);
			break;
		}

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

