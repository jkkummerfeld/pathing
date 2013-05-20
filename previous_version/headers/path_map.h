/* Used to store the nodes used in the search
 *
 */

#pragma once

#include "../headers/node.h"
#include "../headers/point.h"

class Path_Map {
 public:
	unsigned short width;
	unsigned short height;
	unsigned short level;

	Path_Map(unsigned short l, unsigned short w, unsigned short h, unsigned int nodes);

	~Path_Map();

	Node* get_node(const Point &p);

	void print(FILE* out);

 private:
	Node** table;
	unsigned int len_table;

	unsigned int hash(const Point& p);
	unsigned int get_pos(const Point& p);
};

