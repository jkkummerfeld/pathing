/*
 */

#include "../headers/cost_map.h"
#include "../headers/point.h"
#include "../headers/search.h"

#include "../headers/euclidean.h"

Euclidean::Euclidean(Cost_Map* base_map, Point& start, Point& goal) : Search(base_map, start, goal) {
}

Euclidean::~Euclidean() {
}

double
Euclidean::heuristic(const Point &pos) {
 	return euclidean_distance(pos.x, pos.y, goal.x, goal.y);
}

