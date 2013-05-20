/*
 */

#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>

#include	"../headers/cost_map.h"

int
main() {
	printf("Simple tests of the map class:\n");

	// Construct a sample map file
	FILE* out = fopen("tmp.txt", "w");
	for (unsigned short i = 0; i < 5; i++)
		fprintf(out, "1.0 2.0 3.0 4.0\n");
	fclose(out);

	Cost_Map base("tmp.txt", 4, 5, 1, 1);
	Cost_Map level1(&base, 3, 3);
	Cost_Map level2(&level1, 6, 6);
	Cost_Map plain(6, 3, 1, 2, 0.25);

	fprintf(stdout, "\nbase:\n");
	base.print(stdout);
	fprintf(stdout, "\nlevel1:\n");
	level1.print(stdout);
	fprintf(stdout, "\nlevel2:\n");
	level2.print(stdout);
	fprintf(stdout, "\nplain:\n");
	plain.print(stdout);
	
	return 0;
}

