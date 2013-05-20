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
#include "../headers/octile.h"
#include "../headers/euclidean.h"
#include "../headers/cucs_heuristic.h"
#include "../headers/boundaries_blocking.h"
#include "../headers/boundaries_non-blocking.h"
#include "../headers/corners_blocking.h"
#include "../headers/corners_non-blocking.h"

int levels = 100;
int xscale = 4;
int yscale = 4;

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

///	printf("UCS\n");
	UCS ucs(&map, start, goal);
	gettimeofday(&pre, NULL); 
	Path path = ucs.search();
	gettimeofday(&post, NULL); 
	double reference_cost = path.length;
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

///	printf("A*\n");
	Manhattan Manhattan(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = Manhattan.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

///	printf("A*\n");
	Euclidean euclidean(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = euclidean.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

///	printf("A*\n");
	Octile octile(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = octile.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

///	printf("Coarse single\n");
	CUCS_Heuristic cucs(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = cucs.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

///	printf("BB\n");
	Boundaries_Blocking bb2(&map, start, goal, 2, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = bb2.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Boundaries_Blocking bb(&map, start, goal, levels, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = bb.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

///	printf("BN\n");
	Boundaries_NonBlocking bnb2(&map, start, goal, 2, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = bnb2.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Boundaries_NonBlocking bnb(&map, start, goal, levels, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = bnb.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

///	printf("CB\n");
	Corners_Blocking cb2(&map, start, goal, 2, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = cb2.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Corners_Blocking cb(&map, start, goal, levels, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = cb.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

///	printf("CN\n");
	Corners_NonBlocking cnb2(&map, start, goal, 2, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = cnb2.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);

	Corners_NonBlocking cnb(&map, start, goal, levels, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = cnb.search();
	gettimeofday(&post, NULL); 
	check_and_update(start, goal, pre, post, reference_cost, path.length, times);
}

int
main(int argc, char** argv) {
	if (argc < 9) {
		printf("Usage:\n%s <map_file> <width> <height> <x map scale> <y map scale> <levels> <xscale> <yscale> [<start x> <start y> <goal x> <goal y>]\n", argv[0]);
		return 0;
	}
	unsigned short w;
	unsigned short h;
	unsigned short sx;
	unsigned short sy;
	sscanf(argv[2], "%hu", &w);
	sscanf(argv[3], "%hu", &h);
	sscanf(argv[4], "%hu", &sx);
	sscanf(argv[5], "%hu", &sy);
	sscanf(argv[6], "%d", &levels);
	sscanf(argv[7], "%d", &xscale);
	sscanf(argv[8], "%d", &yscale);
	unsigned short start_x = 0;
	unsigned short start_y = 0;
	unsigned short goal_x = 0;
	unsigned short goal_y = 0;
	if (argc == 13) {
		sscanf(argv[9], "%hu", &start_x);
		sscanf(argv[10], "%hu", &start_y);
		sscanf(argv[11], "%hu", &goal_x);
		sscanf(argv[12], "%hu", &goal_y);
	}

	char* map_filename = argv[1];
	Cost_Map map(map_filename, w, h, sx, sy);
	printf("Read map\n");
	unsigned short max_width = map.width;
	unsigned short max_height = map.height;
	// Add random threat
///	for (int i = 0; i < 10000; i++) {
///		unsigned short x = rand() % max_width;
///		unsigned short y = rand() % max_height;
///		unsigned short r = rand() % 50;
///		double value = rand() % 100;
///		map.add_circular_threat(x, y, r, value);
///	}
///	fprintf(stderr, "Added threat\n");
	map.print_small(stdout);
	fflush(stdout);

	vector<double> averages;
	Point start(0, 0, 0);
	Point goal(0, 0, 0);
	int iters = 5;
	if (argc == 9)
		iters = 100;
	for (int n = 0; n < iters; n++) {
		if (argc == 9) {
			// Get random locations
			while (true) {
				start.x = rand() % max_width;
				start.y = rand() % max_height;
				goal.x = rand() % max_width;
				goal.y = rand() % max_height;
				if ((map.get_map_val(start.x, start.y) != HUGE_VAL) &&
						(map.get_map_val(goal.x, goal.y) != HUGE_VAL) &&
						(abs(start.x - goal.x) + abs(start.y - goal.y) > 100)) {
					printf("UCS test\n");
					UCS ucs(&map, start, goal);
					Path path = ucs.search();
					if (path.length > 0)
						break;
				}
			}
		} else {
			start.x = start_x;
			start.y = start_y;
			goal.x = goal_x;
			goal.y = goal_y;
		}
		printf("(%d, %d) to (%d, %d)      ", start.x, start.y, goal.x, goal.y);

		vector<int> times;
		run_test(map, start, goal, times);
		for (unsigned int i = 0; i < times.size(); i++)
			printf("%d   ", times[i]);
		if (averages.size() == 0) {
			for (unsigned int i = 0; i < times.size(); i++)
				averages.push_back(times[i]);
		} else {
			for (unsigned int i = 0; i < times.size(); i++)
				averages[i] += (times[i] - averages[i]) / n;
		}
		printf("\n");
	}
	for (unsigned int i = 0; i < averages.size(); i++)
		printf("%.2lf\n", averages[i] / 1000);
	return 0;
}

