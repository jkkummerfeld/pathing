/*
 */

#include "../headers/cost_map.h"
#include "../headers/point.h"
#include "../headers/search.h"

#include "../headers/manhattan.h"

Manhattan::Manhattan(Cost_Map* base_map, Point& start, Point& goal) : Search(base_map, start, goal) {
}

Manhattan::~Manhattan() {
}

double
Manhattan::heuristic(const Point &pos) {
	return manhattan_distance(pos.x, pos.y, goal.x, goal.y);
}

