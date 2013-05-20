/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include "../headers/node.h"
#include "../headers/cost_map.h"
#include "../headers/path.h"
#include "../headers/cucs_heuristic.h"

int
main(int argc, char** argv) {
	if (argc != 10) {
		printf("Usage:\n%s <width> <height> <x_scal> <y_scale> <map_file> <start_x> <start_y> <goal_x> <goal_y>\n", argv[0]);
		return 0;
	}

	Point start(0, 0, 0);
	Point goal(0, 0, 0);
	short width, height;
	unsigned int uval;
	unsigned int xscale;
	unsigned int yscale;
	sscanf(argv[1], "%hd", &width);
	sscanf(argv[2], "%hd", &height);
	sscanf(argv[3], "%u", &xscale);
	sscanf(argv[4], "%u", &yscale);
	sscanf(argv[6], "%u", &uval); start.x = uval;
	sscanf(argv[7], "%u", &uval); start.y = uval;
	sscanf(argv[8], "%u", &uval); goal.x = uval;
	sscanf(argv[9], "%u", &uval); goal.y = uval;

	Cost_Map map(argv[5], width, height, xscale, yscale);

	CUCS_Heuristic search = CUCS_Heuristic(&map, start, goal);
	Path path = search.search();
	path.print(stdout);

	return 0;
}

