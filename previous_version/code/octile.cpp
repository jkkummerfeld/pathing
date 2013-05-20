/*
 */

#include "../headers/cost_map.h"
#include "../headers/point.h"
#include "../headers/search.h"

#include "../headers/octile.h"

Octile::Octile(Cost_Map* base_map, Point& start, Point& goal) : Search(base_map, start, goal) {
}

Octile::~Octile() {
}

double
Octile::heuristic(const Point &pos) {
	return octile_distance(pos.x, pos.y, goal.x, goal.y);
}

