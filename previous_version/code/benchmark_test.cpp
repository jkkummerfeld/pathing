/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>
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

void
check_and_update(Point &start, Point &goal, struct timeval pre, struct timeval post, double reference_cost, double cost, vector<int> &times) {
	if (fabs(cost - reference_cost) > 1e-5) {
		printf("Wrong cost on:\n");
		start.print(stdout);
		goal.print(stdout);
	}
	int pre_to_post = 1000000 * (post.tv_sec - pre.tv_sec) + post.tv_usec - pre.tv_usec;
	times.push_back(pre_to_post);
}

void
run_test(Cost_Map &map, Point &start, Point &goal, vector<int> &times) {
	struct timeval pre;
	struct timeval post;

	UCS ucs(&map, start, goal);
	gettimeofday(&pre, NULL); 
	Path path = ucs.search();
	gettimeofday(&post, NULL); 
	printf("Path length: %lf\n", path.length);
	if (path.length < 500)
		return;
	double reference_cost = path.length;
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Manhattan manhattan(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = manhattan.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Euclidean euclidean(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = euclidean.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Octile octile(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = octile.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	CUCS_Heuristic cucs(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = cucs.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Boundaries_Blocking bb(&map, start, goal, 3, 4, 4);
	gettimeofday(&pre, NULL); 
	path = bb.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Boundaries_NonBlocking bnb(&map, start, goal, 3, 4, 4);
	gettimeofday(&pre, NULL); 
	path = bnb.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Corners_Blocking cb(&map, start, goal, 3, 4, 4);
	gettimeofday(&pre, NULL); 
	path = cb.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Corners_NonBlocking cnb(&map, start, goal, 3, 4, 4);
	gettimeofday(&pre, NULL); 
	path = cnb.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);
}

int
main(int argc, char** argv) {
	if (argc != 3) {
		printf("Usage:\n%s <scenario_file> <map_file>\n", argv[0]);
		return 0;
	}

	char* map_filename = argv[2];
	Cost_Map map(map_filename);

	char* scenario_filename = argv[1];
	FILE* scenario_file = fopen(scenario_filename, "r");
	char c;
	while ((c = getc(scenario_file)) != '\n') {
	}
	unsigned short start_x;
	unsigned short start_y;
	unsigned short goal_x;
	unsigned short goal_y;
	vector<double> averages;
	int n = 0;
	double octile_cost;
	while (5 == fscanf(scenario_file, "%*d %*s %*d %*d %hu %hu %hu %hu %lf", &start_x, &start_y, &goal_x, &goal_y, &octile_cost)) {
		n++;
		printf("%d     ", n);
		Point start(start_x, start_y, 0);
		Point goal(goal_x, goal_y, 0);
		vector<int> times;
		run_test(map, start, goal, times);
		if (averages.size() == 0) {
			for (unsigned int i = 0; i < times.size(); i++) {
				averages.push_back(times[i]);
				printf("%d  ", times[i]);
			}
		} else {
			for (unsigned int i = 0; i < times.size(); i++) {
				averages[i] += (times[i] - averages[i]) / n;
				printf("%d  ", times[i]);
			}
		}
		printf("\n");
	}
	for (unsigned int i = 0; i < averages.size(); i++)
		printf("%lf\n", averages[i]);
	return 0;
}

