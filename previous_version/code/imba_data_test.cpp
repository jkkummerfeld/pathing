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

int levels = 5;
int xscale = 2;
int yscale = 2;
int reopen = -1;

void
check_and_update(Point &start, Point &goal, int nodes, struct timeval pre, struct timeval post, double reference_cost, double cost, vector<int> &times, vector<int>& expanded) {
	if (fabs(cost - reference_cost) > 1e-8) {
		printf("Cost out: %lf, %lf     ie %lf\n", cost, reference_cost, cost/reference_cost);
		fprintf(stderr, "Cost out: %lf, %lf     ie %lf\n", cost, reference_cost, cost/reference_cost);
	}
	int pre_to_post = 1000000 * (post.tv_sec - pre.tv_sec) + post.tv_usec - pre.tv_usec;
	times.push_back(pre_to_post);
	expanded.push_back(nodes);
}

void
run_test(Cost_Map &map, Point &start, Point &goal, vector<int> &times, vector<int> &expanded) {
	struct timeval pre;
	struct timeval post;

	UCS ucs(&map, start, goal);
	gettimeofday(&pre, NULL); 
	Path path = ucs.search();
	gettimeofday(&post, NULL); 
	double reference_cost = path.length;
	int nodes = ucs.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
///	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);

	Manhattan manhattan(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = manhattan.search();
	gettimeofday(&post, NULL); 
	nodes = manhattan.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
///	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);

	Euclidean euclidean(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = euclidean.search();
	gettimeofday(&post, NULL); 
	nodes = euclidean.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
///	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);

	Octile octile(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = octile.search();
	gettimeofday(&post, NULL); 
	nodes = octile.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
///	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);

	CUCS_Heuristic cucs(&map, start, goal);
	gettimeofday(&pre, NULL); 
	path = cucs.search();
	gettimeofday(&post, NULL); 
	nodes = cucs.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
///	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);

	Boundaries_Blocking bb1(&map, start, goal, 2, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = bb1.search();
	gettimeofday(&post, NULL); 
	nodes = bb1.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
	Boundaries_Blocking bb(&map, start, goal, levels, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = bb.search();
	gettimeofday(&post, NULL); 
	nodes = bb.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);

	Boundaries_NonBlocking bnb1(&map, start, goal, 2, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = bnb1.search();
	gettimeofday(&post, NULL); 
	nodes = bnb1.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
	Boundaries_NonBlocking bnb(&map, start, goal, levels, xscale, yscale);
	gettimeofday(&pre, NULL); 
	path = bnb.search();
	gettimeofday(&post, NULL); 
	nodes = bnb.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);

	Corners_Blocking cb1(&map, start, goal, 2, xscale, yscale);
	if (reopen > 0)
		cb1.reopen_margin = HUGE_VAL;
	gettimeofday(&pre, NULL); 
	path = cb1.search();
	gettimeofday(&post, NULL); 
	nodes = cb1.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
	Corners_Blocking cb(&map, start, goal, levels, xscale, yscale);
	if (reopen > 0)
		cb.reopen_margin = HUGE_VAL;
	gettimeofday(&pre, NULL); 
	path = cb.search();
	gettimeofday(&post, NULL); 
	nodes = cb.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);

	Corners_NonBlocking cnb1(&map, start, goal, 2, xscale, yscale);
	if (reopen > 0)
		cnb1.reopen_margin = HUGE_VAL;
	gettimeofday(&pre, NULL); 
	path = cnb1.search();
	gettimeofday(&post, NULL); 
	nodes = cnb1.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
	Corners_NonBlocking cnb(&map, start, goal, levels, xscale, yscale);
	if (reopen > 0)
		cnb.reopen_margin = HUGE_VAL;
	gettimeofday(&pre, NULL); 
	path = cnb.search();
	gettimeofday(&post, NULL); 
	nodes = cnb.expanded;
	check_and_update(start, goal, nodes, pre, post, reference_cost, path.length, times, expanded);
}

int
main(int argc, char** argv) {
	if (argc < 2) {
		printf("Usage:\n%s <levels> <xs> <ys> <reopen? 0=yes or 1=no> <scenario_files>\n", argv[0]);
		return 0;
	}
	sscanf(argv[1], "%d", &levels);
	sscanf(argv[2], "%d", &xscale);
	sscanf(argv[3], "%d", &yscale);
	sscanf(argv[4], "%d", &reopen);
	printf("Running with %d %d %d\n", levels, xscale, yscale);

	vector<double> averages;
	vector<double> averages_expanded;
	int n = 0;
	for (int scenario = 5; scenario < argc; scenario++) {
		FILE* map_data = fopen(argv[scenario], "r");

		char buf[100];
		int index = 0;
		char c;
		while ((c = getc(map_data)) != '\n') {
			buf[index] = c;
			index += 1;
		}
		buf[index] = '\0';
	///	printf("%s\n", buf);

		int width, height;
		if (2 != sscanf(buf, "%*s %*s %d %*s %d", &width, &height)) {
			fprintf(stderr, "could not read width or height\n");
			return 1;
		}
		c = getc(map_data);
	///	putc(c, stdout);
		c = getc(map_data);
	///	putc(c, stdout);

		index = 0;
		while ((c = getc(map_data)) != '\n') {
			buf[index] = c;
			index += 1;
		}
		buf[index] = '\0';
	///	printf("%s\n", buf);

		int sx, sy, gx, gy;
		if (2 != sscanf(buf, "%d %d", &sx, &sy)) {
			fprintf(stderr, "could not read start pos\n");
			return 1;
		}

		index = 0;
		while ((c = getc(map_data)) != '\n') {
			buf[index] = c;
			index += 1;
		}
		buf[index] = '\0';
	///	printf("%s\n", buf);

		if (2 != sscanf(buf, "%d %d", &gx, &gy)) {
			fprintf(stderr, "could not read start pos\n");
			return 1;
		}
		c = getc(map_data);
	///	putc(c, stdout);
		c = getc(map_data);
	///	putc(c, stdout);
		while ((c = getc(map_data)) != '\n') {
	///		putc(c, stdout);
		}

		Cost_Map map(map_data, width, height);
		fclose(map_data);

		Point start(sx, sy, 0);
		Point goal(gx, gy, 0);
///		for (int iter = 0; iter < 5; iter++) {
		for (int iter = 0; iter < 1; iter++) {
			vector<int> times;
			vector<int> expanded;
			run_test(map, start, goal, times, expanded);
			if (n == 0) {
				for (unsigned int i = 0; i < times.size(); i++) {
					averages.push_back(times[i]);
					printf("%d  ", times[i]);
					fprintf(stderr, "%.0lf  ", averages[i]);
				}
				printf("\n");
				fprintf(stderr, "\n");
				for (unsigned int i = 0; i < expanded.size(); i++) {
					averages_expanded.push_back(expanded[i]);
					printf("%d  ", expanded[i]);
					fprintf(stderr, "%.0lf  ", averages_expanded[i]);
				}
				printf("\n");
				fprintf(stderr, "\n");
				fprintf(stderr, "\n");
			} else {
				for (unsigned int i = 0; i < times.size(); i++) {
					averages[i] += (times[i] - averages[i]) / n;
					printf("%d  ", times[i]);
					fprintf(stderr, "%.0lf  ", averages[i]);
				}
				printf("\n");
				fprintf(stderr, "\n");
				for (unsigned int i = 0; i < expanded.size(); i++) {
					averages_expanded[i] += (expanded[i] - averages_expanded[i]) / n;
					printf("%d  ", expanded[i]);
					fprintf(stderr, "%.0lf  ", averages_expanded[i]);
				}
				printf("\n");
				fprintf(stderr, "\n");
				fprintf(stderr, "\n");
			}
			n += 1;
		}
	}
	printf("Times: ");
	for (unsigned int i = 0; i < averages.size(); i++)
		printf("%lf ", averages[i]);
	printf("\nExpanded: ");
	for (unsigned int i = 0; i < averages.size(); i++)
		printf("%lf ", averages_expanded[i]);
	printf("\n");
	return 0;
}

