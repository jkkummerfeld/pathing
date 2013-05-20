/*
 */

#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>

#include	"../headers/path_map.h"
#include	"../headers/point.h"
#include	"../headers/node.h"

int
main() {
	printf("Simple tests of the path map class:\n");

	fprintf(stdout, "\nempty:\n");
	Path_Map map(0, 3, 3, 25);
	map.print(stdout);

	fprintf(stdout, "\ntop left:\n");
	Point* point = new Point(0, 0);
	Node* node = map.get_node(point);
	node->g = 1.0;
	point->x = 1;
	map.print(stdout);

	fprintf(stdout, "\n3x3 full:\n");
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			point->x = i;
			point->y = j;
			Node* node = map.get_node(point);
			node->g = i + (j / 10.0);
		}
	}
	map.print(stdout);
	
	return 0;
}

