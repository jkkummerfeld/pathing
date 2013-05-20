/*
 */

#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>
#include <vector>

#include	"../headers/cost_map.h"
#include	"../headers/point.h"
#include	"../headers/unit.h"

Cost_Map::Cost_Map(const char* filename, unsigned short w, unsigned short h, unsigned short sx, unsigned short sy) {
	x_scale = sx;
	y_scale = sy;
	full_width = w * sx;
	full_height = h * sy;
	width = w;
	height = h;

	/* Read the map, storing only one value per scale x scale square */
	FILE* map_data = fopen(filename, "r");
	map = (double**) malloc(sizeof(double*) * width);
	for (unsigned short i = 0; i < width; i++) {
		map[i] = (double*) malloc(sizeof(double) * height);
	}
	double val;
	double prev = -1;
	uniform = true;
	for (unsigned int j = 0; j < height; j++) {
		for (unsigned int i = 0; i < width; i++) {
			if (fscanf(map_data, "%lf", &val) != 1)
				fprintf(stderr, "Cost_Map reading error\n");
			if (val < 0)
				map[i][j] = HUGE_VAL;
			else
				map[i][j] = val;
			if ((i > 0 || j > 0) && val != prev)
				uniform = false;
			prev = val;
		}
	}
	fclose(map_data);
}

Cost_Map::Cost_Map(FILE* data, unsigned short w, unsigned short h) {
	x_scale = 1;
	y_scale = 1;
	full_width = w;
	full_height = h;
	width = w;
	height = h;
	char c;

	/* Read the map, storing only one value per scale x scale square */
	map = (double**) malloc(sizeof(double*) * width);
	for (unsigned short i = 0; i < width; i++) {
		map[i] = (double*) malloc(sizeof(double) * height);
	}
	char buf[100];
	int index;
	int x, y;
	while (true) {
		bool walkable = false;
		index = 0;
		while ((c = getc(data)) != '\n') {
			buf[index] = c;
			index++;
			if (c == 't')
				walkable = true;
		}
		buf[index] = '\0';
		if (index < 3)
			break;
		sscanf(buf, "%d %d", &x, &y);
		if (walkable)
			map[x][y] = 1.0;
		else
			map[x][y] = HUGE_VAL;
	}

	// read the 'threat' line
	index = 0;
	while ((c = getc(data)) != '\n') {
		buf[index] = c;
		index++;
	}
	buf[index] = '\0';

	// read threat
	while (true) {
		index = 0;
		while ((c = getc(data)) != '\n') {
			if (c == EOF)
				break;
			buf[index] = c;
			index++;
		}
		if (c == EOF)
			break;
		buf[index] = '\0';
		double threat;
		sscanf(buf, "%d %d %lf", &x, &y, &threat);
		if (threat > 0)
			map[x][y] = threat;
	}

	uniform = false;
}

Cost_Map::Cost_Map(const char* map_data, unsigned short w, unsigned short h) {
	x_scale = 1;
	y_scale = 1;
	full_width = w;
	full_height = h;
	width = full_width;
	height = full_height;
	uniform = false;
	char c;

	/* Read the map, storing only one value per scale x scale square */
	map = (double**) malloc(sizeof(double*) * width);
	for (unsigned short i = 0; i < width; i++)
		map[i] = (double*) malloc(sizeof(double) * height);
	int index = 0;
	for (unsigned int j = 0; j < height; j++) {
		for (unsigned int i = 0; i < width; i++) {
			c = map_data[index];
			index += 1;
			switch (c) {
				case '.':
					map[i][j] = 1;
					break;
				case 'W':
					map[i][j] = HUGE_VAL;
					break;
			}
		}
	}
}

Cost_Map::Cost_Map(const char* filename, bool contains_numbers) {
	x_scale = 1;
	y_scale = 1;
	FILE* map_data = fopen(filename, "r");
	char c;
	if (contains_numbers) {
		if (1 != fscanf(map_data, "%hu", &full_width))
			fprintf(stderr, "Cost map height reading error from file\n");
		if (1 != fscanf(map_data, "%hu", &full_height))
			fprintf(stderr, "Cost map width reading error from file\n");
	} else {
		while ((c = getc(map_data)) != '\n') {
			putc(c, stdout);
		}
		if (1 != fscanf(map_data, "%*s %hu", &full_height))
			fprintf(stderr, "Cost map height reading error\n");
		if (1 != fscanf(map_data, "%*s %hu", &full_width))
			fprintf(stderr, "Cost map width reading error\n");
		c = getc(map_data);
		while ((c = getc(map_data)) != '\n') {
		}
	}
	width = full_width;
	height = full_height;

	/* Read the map, storing only one value per scale x scale square */
	map = (double**) malloc(sizeof(double*) * width);
	for (unsigned short i = 0; i < width; i++) {
		map[i] = (double*) malloc(sizeof(double) * height);
	}
	double val = HUGE_VAL;
	double prev = -1;
	uniform = true;
	for (unsigned int j = 0; j < height; j++) {
		for (unsigned int i = 0; i < width; i++) {
			if (contains_numbers) {
				if (1 != fscanf(map_data, "%lf", &val))
					fprintf(stderr, "Error while reading map from file\n");
				map[i][j] = val;
			} else {
				c = getc(map_data);
				switch (c) {
					// passable terrain
					case '.':
					case 'G':
					// Swamp (passable from regular terrain)
					case 'S':
						map[i][j] = 1;
						val = 1;
						break;
					// out of bounds
					case '@':
					case 'O':
					// Water (traversable, but not passable from terrain)
					case 'W':
					// Trees (unpassable)
					case 'T':
						map[i][j] = HUGE_VAL;
						val = HUGE_VAL;
						break;
				}
			}
			if ((i > 0 || j > 0) && val != prev)
				uniform = false;
			prev = val;
		}
		if (contains_numbers) {
		} else {
			c = getc(map_data);
			if (c != '\n') {
				fprintf(stderr, "Error while reading map\n");
				return;
			}
		}
	}
	fclose(map_data);
}

Cost_Map::Cost_Map(unsigned short w, unsigned short h, unsigned short sx, unsigned short sy, double value) {
	x_scale = sx;
	y_scale = sy;
	full_width = w * sx;
	full_height = h * sy;
	width = w;
	height = h;

	/* Read the map, storing only one value per scale x scale square */
	map = (double**) malloc(sizeof(double*) * width);
	for (unsigned short i = 0; i < width; i++) {
		map[i] = (double*) malloc(sizeof(double) * height);
	}
	uniform = true;
	for (unsigned int j = 0; j < height; j++) {
		for (unsigned int i = 0; i < width; i++) {
			map[i][j] = value;
		}
	}
}

Cost_Map::Cost_Map(Cost_Map* base, unsigned short sx, unsigned short sy) {
	x_scale = sx;
	y_scale = sy;
	full_width = base->full_width;
	full_height = base->full_height;
	width = full_width / sx;
	height = full_height / sy;
	if (full_width % sx != 0)
		width++;
	if (full_height % sy != 0)
		height++;

	map = (double**) malloc(sizeof(double*) * width);
	for (unsigned short i = 0; i < width; i++) {
		map[i] = (double*) malloc(sizeof(double) * height);
		for (unsigned short j = 0; j < height; j++)
			map[i][j] = HUGE_VAL;
	}
	
	/* The selection function
	 * TODO: Generalise to allow rand or min
	 */
	if (base->x_scale > x_scale || base->y_scale > y_scale) {
		// This can only happen when going from the lowest level to the 2nd lowest
		// (e.g. if the map file is very coarse, coarser than a block)
		for (unsigned short i = 0; i < base->full_width; i++) {
			for (unsigned short j = 0; j < base->full_height; j++) {
				unsigned short x = i / x_scale;
				unsigned short lx = i / base->x_scale;
				unsigned short y = j / y_scale;
				unsigned short ly = j / base->y_scale;
				if (base->map[lx][ly] < map[x][y]) {
					map[x][y] = base->map[lx][ly];
				}
			}
		}
	} else {
		for (unsigned short i = 0; i < base->width; i++) {
			for (unsigned short j = 0; j < base->height; j++) {
				unsigned short x = (i * base->x_scale) / x_scale;
				unsigned short y = (j * base->y_scale) / y_scale;
				if (base->map[i][j] < map[x][y]) {
					map[x][y] = base->map[i][j];
				}
			}
		}
	}

	uniform = true;
	double prev = -1;
	for (unsigned short i = 0; i < width; i++) {
		for (unsigned short j = 0; j < height; j++) {
			if ((i > 0 || j > 0) && prev != map[i][j])
				uniform = false;
			prev = map[i][j];
		}
	}
}

Cost_Map::~Cost_Map() {
	for (unsigned short i = 0; i < width; i++)
		free(map[i]);
	free(map);
}

void
Cost_Map::add_circular_threat(unsigned short x, unsigned short y, unsigned short r, double value) {
	for (unsigned short i = 0; i < width; i++) {
		for (unsigned short j = 0; j < height; j++) {
			short dx = x - i;
			short dy = y - j;
			if (dx*dx + dy*dy < r*r)
				map[i][j] += value;
		}
	}
}

double
Cost_Map::get_map_val(unsigned short x, unsigned short y) {
	if (x > width * x_scale || y > height * y_scale)
		return -1;
	return map[x / x_scale][y / y_scale];
}

double
Cost_Map::edge_cost(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) {
	if (x0 == x1 && y0 == y1)
		return 0;
	if ((x0 - x1) != 0 && (y0 - y1) != 0) {
		return HUGE_VAL;
	}
	if (x0 > width * x_scale || y0 > height * y_scale ||
		x1 > width * x_scale || y1 > height * y_scale)
		return -1;
	short dx = x1 - x0;
	short dy = y1 - y0;
	double val1 = HUGE_VAL;
	double val2 = HUGE_VAL;
	if (dx < 0) {
		if (y0 < height * y_scale)
			val1 = get_map_val(x1, y1);
		if (y0 > 0)
			val2 = get_map_val(x1, y1 - 1);
	} else if (dx > 0) {
		if (y0 < height * y_scale)
			val1 = get_map_val(x0, y0);
		if (y0 > 0)
			val2 = get_map_val(x0, y0 - 1);
	} else if (dy < 0) {
		if (x0 < width * x_scale)
			val1 = get_map_val(x1, y1);
		if (x0 > 0)
			val2 = get_map_val(x1 - 1, y1);
	} else if (dy > 0) {
		if (x0 < width * x_scale)
			val1 = get_map_val(x0, y0);
		if (x0 > 0)
			val2 = get_map_val(x0 - 1, y0);
	}

	double ans = val1;
	if (val2 < ans)
		ans = val2;
	if (ans != HUGE_VAL) {
		if (dx != 0)
			return ans * abs(dx);
		else
			return ans * abs(dy);
	} else
		return ans;
}

double
Cost_Map::edge_cost(const Point &from, const Point &to) {
	return edge_cost(from.x, from.y, to.x, to.y);
}

void
Cost_Map::print(FILE* out) {
	fprintf(out, "Cost map %dx%d\n", full_width, full_height);
	fprintf(out, "Stored at scale %d by %d, using %dx%d = %d boxes\n", x_scale, y_scale, width, height, width * height);
	if (uniform)
		fprintf(out, "Uniform\n");
	else
		fprintf(out, "Not uniform\n");
	for (unsigned short j = 0; j < height; j++) {
		for (unsigned short i = 0; i < width; i++)
			fprintf(out, "%.1lf ", map[i][j]);
		fprintf(out, "\n");
	}
}

void
Cost_Map::print_small(FILE* out) {
	fprintf(out, "Cost map %dx%d\n", full_width, full_height);
	fprintf(out, "Stored at scale %d by %d, using %dx%d = %d boxes\n", x_scale, y_scale, width, height, width * height);
	if (uniform)
		fprintf(out, "Uniform\n");
	else
		fprintf(out, "Not uniform\n");
	double max_non_wall = 0.0;
	for (unsigned short j = 0; j < full_height; j++) {
		for (unsigned short i = 0; i < full_width; i++) {
			double val = get_map_val(i, j);
			if (val != HUGE_VAL && val > max_non_wall)
				max_non_wall = val;
		}
	}

	for (unsigned short j = 0; j < full_height; j += 10) {
		for (unsigned short i = 0; i < full_width; i += 10) {
			double val = get_map_val(i, j);
			if (val == HUGE_VAL)
				fprintf(out, " ");
			else if (val < 1.01)
				fprintf(out, ".");
			else if (val / max_non_wall < 0.25)
				fprintf(out, ":");
			else if (val / max_non_wall < 0.5)
				fprintf(out, "o");
			else if (val / max_non_wall < 0.75)
				fprintf(out, "O");
			else
				fprintf(out, "@");
		}
		fprintf(out, "\n");
	}
}

void
Cost_Map::add_threat(vector<Unit> &units, int player) {
	for (unsigned int i = 0; i < units.size(); i++) {
///		units[i].print(stdout);
		unsigned short y = units[i].pos.x;
		unsigned short x = units[i].pos.y;
		if (units[i].player == player) {
			// Mark their occupied space as excessively bad
			for (int yp = y - units[i].up ; yp <= y + units[i].down ; yp++) {
				if (0 <= yp && yp < height) {
					for (int xp = x - units[i].left ; xp <= x + units[i].right ; xp++) {
						if (0 <= xp && xp < width && map[xp][yp] != HUGE_VAL) {
							map[xp][yp] = HUGE_VAL;
						}
					}
				}
			}

			// fill in the circular range
			units[i].range *= 1.5;
			for (int yp = y - units[i].range ; yp <= y + units[i].range ; yp++) {
				if (0 <= yp && yp < height) {
					for (int xp = x - units[i].range ; xp <= x + units[i].range ; xp++) {
						if (0 <= xp && xp < width) {
							if (units[i].range * units[i].range >= ((x - xp) * (x - xp) + (y - yp) * (y - yp))) {
								if (map[xp][yp] != HUGE_VAL)
									map[xp][yp] += units[i].damage / 5;
							}
						}
					}
				}
			}
		}
	}
}

double
Cost_Map::average_cost() {
	double count = 0;
	double average = 0;
	for (unsigned short j = 0; j < height; j += 1) {
		for (unsigned short i = 0; i < width; i += 1) {
			if (map[i][j] != HUGE_VAL) {
				average *= count / (count + 1);
				average += map[i][j] / (count + 1);
				count++;
			}
		}
	}
	return average;
}

