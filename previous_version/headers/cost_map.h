/* Used to store the costs of locations on the map
 *
 * Here the map is treated as a collection of boxes, with a value in each box
 * Maps can be constructed directly from a file, empty, or from another map
 */

#include <cstdio>
#include <vector>

#include "point.h"
#include "unit.h"

#pragma once

using namespace std;

class Cost_Map {
 public:
	unsigned short width;
	unsigned short height;
	unsigned short full_width;
	unsigned short full_height;
	unsigned short x_scale;
	unsigned short y_scale;
	bool uniform;

	/* Construct a map from a file in the format of the site http://movingai.com/benchmarks/index.html
	 */
	Cost_Map(const char* filename, bool contains_numbers=false);

	/* Construct a map in my own format, for processing real games
	 */
	Cost_Map(const char* map_data, unsigned short w, unsigned short h);

	Cost_Map(FILE* map_data, unsigned short w, unsigned short h);

	/* Construct a map from a file
	 * w and h refer to the width and height of the matrix of values in the file
	 * sx and sy define the size of each box
	 * the map is recorded as given, but positions are treated as if each
	 * square is a sx by sy group of squares
	 */
	Cost_Map(const char* filename, unsigned short w, unsigned short h, unsigned short sx, unsigned short sy);

	Cost_Map(unsigned short w, unsigned short h, unsigned short sx, unsigned short sy, double value);

	/* Construct a map by coarsening another map
	 * The scale should be relative to the actual full map
	 */
	Cost_Map(Cost_Map* base, unsigned short sx, unsigned short sy);

	~Cost_Map();

	void add_circular_threat(unsigned short x, unsigned short y, unsigned short r, double value);

	double get_map_val(unsigned short x, unsigned short y);
	double edge_cost(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1);
	double edge_cost(const Point &from, const Point &to);

	void print(FILE* out);
	void print_small(FILE* out);

	void add_threat(vector<Unit> &units, int player);
	double average_cost();

 private:
	double** map;
	FILE* threat_info;
};

