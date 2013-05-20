/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>
#include	<time.h>
#include <sys/time.h>
#include	<vector>

#include "../headers/node.h"
#include "../headers/cost_map.h"
#include "../headers/path.h"
#include "../headers/ucs.h"
#include "../headers/manhattan.h"
#include "../headers/euclidean.h"
#include "../headers/octile.h"
#include "../headers/cucs_heuristic.h"
#include "../headers/boundaries_blocking.h"
#include "../headers/boundaries_non-blocking.h"
#include "../headers/corners_blocking.h"
#include "../headers/corners_non-blocking.h"

using namespace std;

int
main(int argc, char** argv) {
	if (argc != 7 && argc != 11) {
		printf("Usage:\n%s <width> <height> <x_scal> <y_scale> <map_file> <iterations> [<start_x> <start_y> <goal_x> <goal_y>]\n", argv[0]);
		return 0;
	}

	Point start(0, 0, 0);
	Point goal(0, 0, 0);
	short width, height;
	unsigned int uval;
	unsigned int xscale;
	unsigned int yscale;
	unsigned int iterations;
	sscanf(argv[1], "%hd", &width);
	sscanf(argv[2], "%hd", &height);
	sscanf(argv[3], "%u", &xscale);
	sscanf(argv[4], "%u", &yscale);
	sscanf(argv[6], "%u", &iterations);
	if (argc == 11) {
		sscanf(argv[7], "%u", &uval); start.x = uval;
		sscanf(argv[8], "%u", &uval); start.y = uval;
		sscanf(argv[9], "%u", &uval); goal.x = uval;
		sscanf(argv[10], "%u", &uval); goal.y = uval;
	}

	Cost_Map map(argv[5], width, height, xscale, yscale);

	for (unsigned int iter = 0; iter < iterations; iter++) {
		if (argc == 7) {
			unsigned int max_width = width * xscale;
			unsigned int max_height = height * yscale;
			start.x = rand() % max_width;
			start.y = rand() % max_height;
			goal.x = rand() % max_width;
			goal.y = rand() % max_height;
		}
		printf("Path (%d %d %d %d)", start.x, start.y, goal.x, goal.y);
		vector<double> costs;

		UCS ucs = UCS(&map, start, goal);
		Path ucs_path = ucs.search();
		costs.push_back(ucs_path.length);
		printf(" cost %lf  \t", ucs_path.length);

		printf("Manhattan");
		Manhattan manhattan = Manhattan(&map, start, goal);
		Path manhattan_path = manhattan.search();
		costs.push_back(manhattan_path.length);

		printf("Octile");
		Octile octile = Octile(&map, start, goal);
		Path octile_path = octile.search();
		costs.push_back(octile_path.length);

		printf("Euclidean");
		Euclidean euclidean = Euclidean(&map, start, goal);
		Path euclidean_path = euclidean.search();
		costs.push_back(euclidean_path.length);

		printf("CUCS   ");
		CUCS_Heuristic cucs_heuristic = CUCS_Heuristic(&map, start, goal);
		cucs_heuristic.coarse_expanded = 0;
		Path cucs_heuristic_path = cucs_heuristic.search();
		costs.push_back(cucs_heuristic_path.length);

		printf("BB   ");
		Boundaries_Blocking boundaries_blocking = Boundaries_Blocking(&map, start, goal, 100, 4, 4);
		boundaries_blocking.coarse_expanded = 0;
		Path boundaries_blocking_path = boundaries_blocking.search();
		costs.push_back(boundaries_blocking_path.length);

		printf("BNB   ");
		Boundaries_NonBlocking boundaries_non_blocking = Boundaries_NonBlocking(&map, start, goal, 100, 4, 4);
		boundaries_non_blocking.coarse_expanded = 0;
		Path boundaries_non_blocking_path = boundaries_non_blocking.search();
		costs.push_back(boundaries_non_blocking_path.length);

		printf("CB    ");
		Corners_Blocking corners_blocking = Corners_Blocking(&map, start, goal, 3, 4, 4);
		corners_blocking.coarse_expanded = 0;
		Path corners_blocking_path = corners_blocking.search();
		costs.push_back(corners_blocking_path.length);

		printf("CNB   ");
		Corners_NonBlocking corners_non_blocking = Corners_NonBlocking(&map, start, goal, 3, 4, 4);
		corners_non_blocking.coarse_expanded = 0;
		Path corners_non_blocking_path = corners_non_blocking.search();
		costs.push_back(corners_non_blocking_path.length);

		printf("\n");
		for (unsigned int i = 0; i < costs.size(); i++) {
			if (fabs(costs[i] - costs[0]) > 1e-6) {
				printf("Costs do not agree.\n");
				for (unsigned int j = 0; j < costs.size(); j++)
					printf("%lf\t", costs[j]);
				printf("\n");
				return 1;
			}
		}
		if (argc == 10)
			break;
	}

	return 0;
}

