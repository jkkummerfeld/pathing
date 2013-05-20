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
#include "../headers/manhattan.h"
#include "../headers/euclidean.h"
#include "../headers/octile.h"
#include "../headers/ucs.h"
#include "../headers/cucs_heuristic.h"
#include "../headers/boundaries_blocking.h"
#include "../headers/boundaries_non-blocking.h"
#include "../headers/corners_blocking.h"
#include "../headers/corners_non-blocking.h"

using namespace std;

int
main(int argc, char** argv) {
	if (argc != 11) {
		fprintf(stderr, "Expected 11 arguments\n");
		return 2;
	}
	Point start(0, 0, 0);
	Point goal(0, 0, 0);
///	short width, height;
	unsigned int uval;
	int slevels = 3;
	int sxscale = 3;
	int syscale = 3;
	int player_num = 0;
	int algorithm = 0;
	sscanf(argv[2], "%u", &uval); start.x = uval;
	sscanf(argv[3], "%u", &uval); start.y = uval;
	sscanf(argv[4], "%u", &uval); goal.x = uval;
	sscanf(argv[5], "%u", &uval); goal.y = uval;
	sscanf(argv[6], "%d", &slevels);
	sscanf(argv[7], "%d", &sxscale);
	sscanf(argv[8], "%d", &syscale);
	sscanf(argv[9], "%d", &player_num);
	sscanf(argv[10], "%d", &algorithm);

	FILE* unit_info = fopen(argv[1], "r");

	// read past the map data
	char c;
	char buf[1000];
	int index = 0;
	while ((c = getc(unit_info)) != '\n') {
		buf[index] = c;
		index++;
	}
	buf[index] = '\0';
	unsigned short full_width;
	unsigned short full_height;
	sscanf(buf, "%hu %hu", &full_width, &full_height);
	char* map_data = (char*) malloc(sizeof(char) * full_width * full_height);
	index = 0;
	for (unsigned int j = 0; j < full_height; j++) {
		for (unsigned int i = 0; i < full_width; i++) {
			map_data[index] = getc(unit_info);
			index += 1;
		}
		getc(unit_info);
	}

	// read unit positions, we want the last set, so read until we get it
	vector<Unit> units;

	int pp0 = 0;
	int pp1 = 0;
	while (true) {
		index = 0;
		while ((c = getc(unit_info)) != '\n') {
			if (c == EOF)
				break;
			buf[index] = c;
			index++;
		}
		if (c == EOF)
			break; // no more data
		buf[index] = '\0';
		unsigned int len_units = 0;
		int read = sscanf(buf, "%u", &len_units);
		if (read != 1)
			printf("Failed to read the number of units\n");
		vector<Unit> nunits;
		int p0 = 0;
		int p1 = 0;
		for (unsigned int i = 0; i < len_units; i++) {
			index = 0;
			while ((c = getc(unit_info)) != '\n') {
				if (c == EOF)
					break;
				buf[index] = c;
				index++;
			}
			buf[index] = '\0';
			if (index == 0) {
				len_units = i;
				break;
			}
			int player, range, damage, left, right, up, down, x, y;
			if (9 != sscanf(buf, "%d %d %d %d %d %d %d %d %d", &player, &range, &damage, &left, &right, &up, &down, &x, &y)) {
				fprintf(stderr, "Error reading unit info\n");
				return 0;
			}
			Unit u;
			u.pos.x = x;
			u.pos.y = y;
			u.player = player;
			if (player == 0)
				p0++;
			else if (player == 1)
				p1++;
			u.range = range;
			u.damage = damage;
			u.left = left;
			u.right = right;
			u.up = up;
			u.down = down;
			units.push_back(u);
		}

		if (p0 < 0.8*pp0 || p1 < 0.8*pp1)
			break; // tide of battle is turning
		units = nunits;
		pp0 = p0;
		pp1 = p1;
	}

	fclose(unit_info);

	Cost_Map map(map_data, full_width, full_height);
	map.add_threat(units, player_num);
///	width = map.width;
///	height = map.height;

	struct timeval pre;
	struct timeval mid;
	struct timeval post;
	int pre_to_mid;
	int mid_to_post;

	for (int i = 0; i < 2; i++) {
		if (algorithm == 0) {
			gettimeofday(&pre, NULL); 
			UCS ucs = UCS(&map, start, goal);
			gettimeofday(&mid, NULL); 
			Path ucs_path = ucs.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("0 (UCS)                    :\t%d\t%d\t%f\t%d\n", pre_to_mid, mid_to_post, ucs_path.length, ucs.expanded);
		} else if (algorithm == 1) {
			gettimeofday(&pre, NULL); 
			Euclidean euclidean = Euclidean(&map, start, goal);
			gettimeofday(&mid, NULL); 
			Path euclidean_path = euclidean.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Euclidean (A*)             :\t%d\t%d\t%f\t%d\n", pre_to_mid, mid_to_post, euclidean_path.length, euclidean.expanded);
		} else if (algorithm == 2) {
			gettimeofday(&pre, NULL); 
			Octile octile = Octile(&map, start, goal);
			gettimeofday(&mid, NULL); 
			Path octile_path = octile.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Octile (A*)             :\t%d\t%d\t%f\t%d\n", pre_to_mid, mid_to_post, octile_path.length, octile.expanded);
		} else if (algorithm == 3) {
			gettimeofday(&pre, NULL); 
			Manhattan manhattan = Manhattan(&map, start, goal);
			gettimeofday(&mid, NULL); 
			Path manhattan_path = manhattan.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Manhattan (A*)             :\t%d\t%d\t%f\t%d\n", pre_to_mid, mid_to_post, manhattan_path.length, manhattan.expanded);
		} else if (algorithm == 4) {
			gettimeofday(&pre, NULL); 
			CUCS_Heuristic cucs_heuristic = CUCS_Heuristic(&map, start, goal);
			gettimeofday(&mid, NULL); 
			cucs_heuristic.coarse_expanded = 0;
			Path cucs_heuristic_path = cucs_heuristic.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Coarse-UCS                 :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, cucs_heuristic_path.length, cucs_heuristic.expanded, cucs_heuristic.coarse_expanded);
		} else if (algorithm == 5) {
			gettimeofday(&pre, NULL); 
			Boundaries_Blocking boundaries_blocking = Boundaries_Blocking(&map, start, goal, slevels, sxscale, syscale);
			gettimeofday(&mid, NULL); 
			boundaries_blocking.coarse_expanded = 0;
			Path boundaries_blocking_path = boundaries_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Boundaries-Blocking        :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, boundaries_blocking_path.length, boundaries_blocking.expanded, boundaries_blocking.coarse_expanded);
		} else if (algorithm == 6) {
			gettimeofday(&pre, NULL); 
			Boundaries_NonBlocking boundaries_non_blocking = Boundaries_NonBlocking(&map, start, goal, slevels, sxscale, syscale);
			gettimeofday(&mid, NULL); 
			boundaries_non_blocking.coarse_expanded = 0;
			Path boundaries_non_blocking_path = boundaries_non_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Boundaries-Non-Blocking    :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, boundaries_non_blocking_path.length, boundaries_non_blocking.expanded, boundaries_non_blocking.coarse_expanded);
		} else if (algorithm == 7) {
			gettimeofday(&pre, NULL); 
			Corners_Blocking corners_blocking = Corners_Blocking(&map, start, goal, slevels, sxscale, syscale);
			gettimeofday(&mid, NULL); 
			corners_blocking.coarse_expanded = 0;
			Path corners_blocking_path = corners_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Corners-Blocking           :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, corners_blocking_path.length, corners_blocking.expanded, corners_blocking.coarse_expanded);
		} else if (algorithm == 8) {
			gettimeofday(&pre, NULL); 
			Corners_NonBlocking corners_non_blocking = Corners_NonBlocking(&map, start, goal, slevels, sxscale, syscale);
			gettimeofday(&mid, NULL); 
			corners_non_blocking.coarse_expanded = 0;
			Path corners_non_blocking_path = corners_non_blocking.search();
			gettimeofday(&post, NULL); 
			pre_to_mid = 1000000 * (mid.tv_sec - pre.tv_sec) + mid.tv_usec - pre.tv_usec;
			mid_to_post = 1000000 * (post.tv_sec - mid.tv_sec) + post.tv_usec - mid.tv_usec;
			printf("Corners-Non-Blocking       :\t%d\t%d\t%f\t%d\t%d\n", pre_to_mid, mid_to_post, corners_non_blocking_path.length, corners_non_blocking.expanded, corners_non_blocking.coarse_expanded);
		}
	}

	return 0;
}

