/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include "../headers/node.h"
#include "../headers/cost_map.h"
#include "../headers/path.h"
#include "../headers/heap.h"
#include "../headers/ucs.h"

UCS::UCS(Cost_Map* base_map, Point& start, Point& goal) :
	Search(base_map, start, goal) {
}

UCS::~UCS() {

}

double
UCS::heuristic(const Point &pos) {
	return 0.0;
}
