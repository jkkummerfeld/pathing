/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>
#include	<time.h>
#include <sys/time.h>
#include <stdlib.h>

#include "../headers/node.h"
#include "../headers/unit.h"
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
	if (argc < 10) {
		printf("Usage:\n%s <width> <height> <x_scal> <y_scale> <map_file> <start_x> <start_y> <goal_x> <goal_y> [levels] [coarse block size X] [coarse block size Y]\n", argv[0]);
		return 0;
	}

	Point start(0, 0, 0);
	Point goal(0, 0, 0);
	short width, height;
	unsigned int uval;
	unsigned int xscale;
	unsigned int yscale;

	int slevels = 3;
	int sxscale = 3;
	int syscale = 3;
	int player_num = 0;
	sscanf(argv[1], "%hd", &width);
	sscanf(argv[2], "%hd", &height);
	sscanf(argv[3], "%u", &xscale);
	sscanf(argv[4], "%u", &yscale);
	sscanf(argv[6], "%u", &uval); start.x = uval;
	sscanf(argv[7], "%u", &uval); start.y = uval;
	sscanf(argv[8], "%u", &uval); goal.x = uval;
	sscanf(argv[9], "%u", &uval); goal.y = uval;
	if (argc > 10)
		sscanf(argv[10], "%d", &slevels);
	if (argc > 11)
		sscanf(argv[11], "%d", &sxscale);
	if (argc > 12)
		sscanf(argv[12], "%d", &syscale);
///	sscanf(argv[13], "%d", &player_num);

///	FILE* unit_info = fopen(argv[5], "r");

///	// read past the map data
///	char c;
///	char buf[1000];
///	int index = 0;
///	while ((c = getc(unit_info)) != '\n') {
///		buf[index] = c;
///		index++;
///	}
///	buf[index] = '\0';
///	unsigned short full_width;
///	unsigned short full_height;
///	sscanf(buf, "%hu %hu", &full_width, &full_height);
///	char* map_data = (char*) malloc(sizeof(char) * full_width * full_height);
///	index = 0;
///	for (unsigned int j = 0; j < full_height; j++) {
///		for (unsigned int i = 0; i < full_width; i++) {
///			map_data[index] = getc(unit_info);
///			index += 1;
///		}
///		getc(unit_info);
///	}

///	// read unit positions, we want the last set, so read until we get it
///	vector<Unit> units;

///	int pp0 = 0;
///	int pp1 = 0;
///	while (true) {
///		index = 0;
///		while ((c = getc(unit_info)) != '\n') {
///			if (c == EOF)
///				break;
///			buf[index] = c;
///			index++;
///		}
///		if (c == EOF)
///			break; // no more data
///		buf[index] = '\0';
///		unsigned int len_units = 0;
///		int read = sscanf(buf, "%u", &len_units);
///		if (read != 1)
///			printf("Failed to read the number of units\n");
///		vector<Unit> nunits;
///		int p0 = 0;
///		int p1 = 0;
///		for (unsigned int i = 0; i < len_units; i++) {
///			index = 0;
///			while ((c = getc(unit_info)) != '\n') {
///				if (c == EOF)
///					break;
///				buf[index] = c;
///				index++;
///			}
///			buf[index] = '\0';
///			if (index == 0) {
///				len_units = i;
///				break;
///			}
///			int player, range, damage, left, right, up, down, x, y;
///			if (9 != sscanf(buf, "%d %d %d %d %d %d %d %d %d", &player, &range, &damage, &left, &right, &up, &down, &x, &y)) {
///				fprintf(stderr, "Error reading unit info\n");
///				return 0;
///			}
///			Unit u;
///			u.pos.x = x;
///			u.pos.y = y;
///			u.player = player;
///			if (player == 0)
///				p0++;
///			else if (player == 1)
///				p1++;
///			u.range = range;
///			u.damage = damage;
///			u.left = left;
///			u.right = right;
///			u.up = up;
///			u.down = down;
///			units.push_back(u);
///		}

///		if (p0 < 0.8*pp0 || p1 < 0.8*pp1)
///			break; // tide of battle is turning
///		units = nunits;
///		pp0 = p0;
///		pp1 = p1;
///	}

///	fclose(unit_info);

	Cost_Map map(argv[5], width, height, xscale, yscale);
///	Cost_Map map(map_data, full_width, full_height);
///	map.add_threat(units, player_num);
	width = map.width;
	height = map.height;
///	for (int x = 0; x < 4096; x += 400) {
///		for (int y = 0; y < 4096; y += 400) {
///			map.add_circular_threat(x, y, rand() % 200, rand() % 100);
///		}
///	}
///	if (map.average_cost() > 2.0) {
///		printf("%lf\n", map.average_cost());
///		map.print_small(stdout);
///		fflush(stdout);
///		Cost_Map map2(map_data, full_width, full_height);
///		map2.add_threat(units, 1);
///		printf("\n\n\n\n\n\n\n\n\n\n%lf\n", map2.average_cost());
///		map2.print_small(stdout);
///		fflush(stdout);
///	}
///	return 0;

///	double best = 0;
///	for (int i = 0; i < 50;) {
///		int sx = rand() % width;
///		int gx = rand() % width;
///		int sy = rand() % height;
///		int gy = rand() % height;
///		int dx = sx - gx;
///		int dy = sy - gy;
///		double dist = sqrt(dx*dx + dy*dy);
///		if (dist > 100) {
///			Point tstart(sx, sy, 0);
///			Point tgoal(gx, gy, 0);
///			A_Star a_star = A_Star(&map, tstart, tgoal);
///			Path a_star_path = a_star.search();
///			double length = a_star_path.length;
///			printf("length %lf dist %lf ratio %lf\n", length, dist, length / dist);
///			if (length / dist > best) {
///				best = length / dist;
///				start.x = sx;
///				goal.x = gx;
///				start.y = sy;
///				goal.y = gy;
///				printf("new best length %lf\n", best);
///			}
///			i++;
///		}
///	}
	printf("Going from (%d %d) to (%d %d)\n", start.x, start.y, goal.x, goal.y);

	printf("Heuristic\n");
	double av_ucs = 0;
	double av_cucs = 0;
	double av_bb = 0;
	double av_bnb = 0;
	double av_cb = 0;
	double av_cnb = 0;

///	for (int i = 0; i < 1; i++) {
	for (int i = 0; i < 10; i++) {
		struct timeval pre;
		struct timeval mid;
		struct timeval post;
		int pre_to_mid;
		int mid_to_post;

		{
			gettimeofday(&pre, NULL); 
			UCS ucs = UCS(&map, start, goal);
			gettimeofday(&mid, NULL); 
			Path ucs_path = ucs.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("0 (UCS)                    :\t%d\t%d\t%f\t%d\n", pre_to_mid, mid_to_post, ucs_path.length, ucs.expanded);
			if (i > 0)
				av_ucs += (mid_to_post - av_ucs) / i;
			fflush(stdout);
		}

		{
			gettimeofday(&pre, NULL); 
			Manhattan manhattan = Manhattan(&map, start, goal);
			gettimeofday(&mid, NULL); 
			Path manhattan_path = manhattan.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Manhattan                  :\t%d\t%d\t%f\t%d\n", pre_to_mid, mid_to_post, manhattan_path.length, manhattan.expanded);
			fflush(stdout);
		}

		{
			gettimeofday(&pre, NULL); 
			Euclidean euclidean = Euclidean(&map, start, goal);
			gettimeofday(&mid, NULL); 
			Path euclidean_path = euclidean.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Euclidean                  :\t%d\t%d\t%f\t%d\n", pre_to_mid, mid_to_post, euclidean_path.length, euclidean.expanded);
			fflush(stdout);
		}

		{
			gettimeofday(&pre, NULL); 
			Octile octile = Octile(&map, start, goal);
			gettimeofday(&mid, NULL); 
			Path octile_path = octile.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Octile                     :\t%d\t%d\t%f\t%d\n", pre_to_mid, mid_to_post, octile_path.length, octile.expanded);
			fflush(stdout);
		}

///		{
///			gettimeofday(&pre, NULL); 
///			CUCS_Heuristic cucs_heuristic = CUCS_Heuristic(&map, start, goal);
///			gettimeofday(&mid, NULL); 
///			cucs_heuristic.coarse_expanded = 0;
///			Path cucs_heuristic_path = cucs_heuristic.search();
///			gettimeofday(&post, NULL); 
///			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
///			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
///			printf("Coarse-UCS                 :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, cucs_heuristic_path.length, cucs_heuristic.expanded, cucs_heuristic.coarse_expanded);
///			if (i > 0)
///				av_cucs += (mid_to_post - av_cucs) / i;
///			fflush(stdout);

///		}

///		for (int tnlevels = 9; tnlevels < 8; tnlevels++) {
		for (int tnlevels = 2; tnlevels < 3; tnlevels++) {
			{
				gettimeofday(&pre, NULL); 
				Boundaries_Blocking boundaries_blocking = Boundaries_Blocking(&map, start, goal, tnlevels, sxscale, syscale);
				gettimeofday(&mid, NULL); 
				boundaries_blocking.coarse_expanded = 0;
				Path boundaries_blocking_path = boundaries_blocking.search();
				gettimeofday(&post, NULL); 
				pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
				mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
				printf("Boundaries-Blocking %d   :\t%d\t%d\t%f\t%d\t%d\n", tnlevels, pre_to_mid, mid_to_post, boundaries_blocking_path.length, boundaries_blocking.expanded, boundaries_blocking.coarse_expanded);

				fflush(stdout);
			}

			{
				gettimeofday(&pre, NULL); 
				Boundaries_NonBlocking boundaries_non_blocking = Boundaries_NonBlocking(&map, start, goal, tnlevels, sxscale, syscale);
				gettimeofday(&mid, NULL); 
				boundaries_non_blocking.coarse_expanded = 0;
				Path boundaries_non_blocking_path = boundaries_non_blocking.search();
				gettimeofday(&post, NULL); 
				pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
				mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
				printf("Boundaries-Non-Blocking %d  :\t%d\t%d\t%f\t%d\t%d\n", tnlevels, pre_to_mid, mid_to_post, boundaries_non_blocking_path.length, boundaries_non_blocking.expanded, boundaries_non_blocking.coarse_expanded);
				fflush(stdout);
			}

			{
				gettimeofday(&pre, NULL); 
				Corners_Blocking corners_blocking = Corners_Blocking(&map, start, goal, tnlevels, sxscale, syscale);
				gettimeofday(&mid, NULL); 
				corners_blocking.coarse_expanded = 0;
				Path corners_blocking_path = corners_blocking.search();
				gettimeofday(&post, NULL); 
				pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
				mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
				printf("Corners-Blocking %d         :\t%d\t%d\t%f\t%d\t%d\n", tnlevels, pre_to_mid, mid_to_post, corners_blocking_path.length, corners_blocking.expanded, corners_blocking.coarse_expanded);
				fflush(stdout);
			}
			{
				gettimeofday(&pre, NULL); 
				Corners_Blocking corners_blocking = Corners_Blocking(&map, start, goal, tnlevels, sxscale, syscale);
				corners_blocking.reopen_margin = HUGE_VAL;
				gettimeofday(&mid, NULL); 
				corners_blocking.coarse_expanded = 0;
				Path corners_blocking_path = corners_blocking.search();
				gettimeofday(&post, NULL); 
				pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
				mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
				printf("Corners-Blocking %d no-repoen      :\t%d\t%d\t%f\t%d\t%d\n", tnlevels, pre_to_mid, mid_to_post, corners_blocking_path.length, corners_blocking.expanded, corners_blocking.coarse_expanded);
				fflush(stdout);
			}

			{
				gettimeofday(&pre, NULL); 
				Corners_NonBlocking corners_non_blocking = Corners_NonBlocking(&map, start, goal, tnlevels, sxscale, syscale);
				gettimeofday(&mid, NULL); 
				corners_non_blocking.coarse_expanded = 0;
				Path corners_non_blocking_path = corners_non_blocking.search();
				gettimeofday(&post, NULL); 
				pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
				mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
				printf("Corners-Non-Blocking %d     :\t%d\t%d\t%f\t%d\t%d\n", tnlevels, pre_to_mid, mid_to_post, corners_non_blocking_path.length, corners_non_blocking.expanded, corners_non_blocking.coarse_expanded);
				fflush(stdout);
			}

			{
				gettimeofday(&pre, NULL); 
				Corners_NonBlocking corners_non_blocking = Corners_NonBlocking(&map, start, goal, tnlevels, sxscale, syscale);
				corners_non_blocking.reopen_margin = HUGE_VAL;
				gettimeofday(&mid, NULL); 
				corners_non_blocking.coarse_expanded = 0;
				Path corners_non_blocking_path = corners_non_blocking.search();
				gettimeofday(&post, NULL); 
				pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
				mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
				printf("Corners-Non-Blocking %d no-reopen  :\t%d\t%d\t%f\t%d\t%d\n", tnlevels, pre_to_mid, mid_to_post, corners_non_blocking_path.length, corners_non_blocking.expanded, corners_non_blocking.coarse_expanded);
				fflush(stdout);
			}
		}

		{
			gettimeofday(&pre, NULL); 
			Boundaries_Blocking boundaries_blocking = Boundaries_Blocking(&map, start, goal, slevels, sxscale, syscale);
			gettimeofday(&mid, NULL); 
			boundaries_blocking.coarse_expanded = 0;
			Path boundaries_blocking_path = boundaries_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Boundaries-Blocking        :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, boundaries_blocking_path.length, boundaries_blocking.expanded, boundaries_blocking.coarse_expanded);
			if (i > 0)
				av_bb += (mid_to_post - av_bb) / i;
			fflush(stdout);

		}

		{
			gettimeofday(&pre, NULL); 
			Boundaries_NonBlocking boundaries_non_blocking = Boundaries_NonBlocking(&map, start, goal, slevels, sxscale, syscale);
			gettimeofday(&mid, NULL); 
			boundaries_non_blocking.coarse_expanded = 0;
			Path boundaries_non_blocking_path = boundaries_non_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Boundaries-Non-Blocking    :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, boundaries_non_blocking_path.length, boundaries_non_blocking.expanded, boundaries_non_blocking.coarse_expanded);
			if (i > 0)
				av_bnb += (mid_to_post - av_bnb) / i;
			fflush(stdout);

		}

		{
			gettimeofday(&pre, NULL); 
			Corners_Blocking corners_blocking = Corners_Blocking(&map, start, goal, slevels, sxscale, syscale);
			gettimeofday(&mid, NULL); 
			corners_blocking.coarse_expanded = 0;
			Path corners_blocking_path = corners_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Corners-Blocking           :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, corners_blocking_path.length, corners_blocking.expanded, corners_blocking.coarse_expanded);
			if (i > 0)
				av_cb += (mid_to_post - av_cb) / i;
			fflush(stdout);

		}
		{
			gettimeofday(&pre, NULL); 
			Corners_Blocking corners_blocking = Corners_Blocking(&map, start, goal, slevels, sxscale, syscale);
			corners_blocking.reopen_margin = HUGE_VAL;
			gettimeofday(&mid, NULL); 
			corners_blocking.coarse_expanded = 0;
			Path corners_blocking_path = corners_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Corners-Blocking no-reopen   :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, corners_blocking_path.length, corners_blocking.expanded, corners_blocking.coarse_expanded);
			if (i > 0)
				av_cb += (mid_to_post - av_cb) / i;
			fflush(stdout);

		}

		{
			gettimeofday(&pre, NULL); 
			Corners_NonBlocking corners_non_blocking = Corners_NonBlocking(&map, start, goal, slevels, sxscale, syscale);
			gettimeofday(&mid, NULL); 
			corners_non_blocking.coarse_expanded = 0;
			Path corners_non_blocking_path = corners_non_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Corners-Non-Blocking       :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, corners_non_blocking_path.length, corners_non_blocking.expanded, corners_non_blocking.coarse_expanded);
			if (i > 0)
				av_cnb += (mid_to_post - av_cnb) / i;
			fflush(stdout);
		}

		{
			gettimeofday(&pre, NULL); 
			Corners_NonBlocking corners_non_blocking = Corners_NonBlocking(&map, start, goal, slevels, sxscale, syscale);
			corners_non_blocking.reopen_margin = HUGE_VAL;
			gettimeofday(&mid, NULL); 
			corners_non_blocking.coarse_expanded = 0;
			Path corners_non_blocking_path = corners_non_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Corners-Non-Blocking no-reopen   :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, corners_non_blocking_path.length, corners_non_blocking.expanded, corners_non_blocking.coarse_expanded);
			if (i > 0)
				av_cnb += (mid_to_post - av_cnb) / i;
			fflush(stdout);
		}

		printf("\n");
	}
///	printf("0 (UCS)                  :\t%f\n", av_ucs);
///	printf("Manhattan (A*)           :\t%f\n", av_a_star);
///	printf("Coarse-UCS               :\t%f\n", av_cucs);
///	printf("Boundaries Blocking      :\t%f\n", av_bb);
///	printf("Boundaries Non-Blocking  :\t%f\n", av_bnb);
///	printf("Corners Blocking         :\t%f\n", av_cb);
///	printf("Corners Non-Blocking     :\t%f\n", av_cnb);

	return 0;
}

