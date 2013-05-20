/*
 */

#include	<vector>

#include "../headers/node.h"
#include "../headers/point.h"

#include "../headers/dependency_map.h"

Dependency_Map::Dependency_Map(unsigned short l, unsigned short w, unsigned short h) : width(w), height(h), level(l) {
	table = (vector<Node*>**) calloc(width*height, sizeof(vector<Node*>*));
///	printf("Table dimensions: %d x %d = %d spaces\n", width, height, width * height);
}

Dependency_Map::~Dependency_Map() {
	for (int j = 0; j < width * height; j++) {
		vector<Node*>* waiters = table[j];
		if (waiters != NULL)
			delete waiters;
	}
	free(table);
}

unsigned int
Dependency_Map::get_pos(const Point& p) {
	return p.x + width * p.y;
}

void
Dependency_Map::add_dependency(Node* waiter, const Node& unknown) {
///	printf("Adding dependency:\n");
///	printf("From\t");
///	waiter->print(stdout);
///	printf("To  \t");
///	unknown.print(stdout);
	unsigned int pos = get_pos(unknown.pos);
///	printf("Pos \t%u\n", pos);
	if (table[pos] == NULL) {
		table[pos] = new vector<Node*>();
///		table[pos]->reserve(10);
	}
	table[pos]->push_back(waiter);
}

vector<Node*>*
Dependency_Map::get_dependencies(const Node& n) {
///	printf("Getting dependencies for\t");
///	n.print(stdout);
	return table[get_pos(n.pos)];
}

void
Dependency_Map::print(FILE* out) {
	Point p(0, 0, level);
	for (unsigned int j = 0; j < height; j++) {
		p.y = j;
		for (unsigned int i = 0; i < width; i++) {
			p.x = i;
			vector<Node*>* waiters = table[get_pos(p)];
			if (waiters == NULL || waiters->size() == 0)
				fprintf(out, "     ");
			else
				fprintf(out, "%4d ", (int)waiters->size());
		}
		fprintf(out, "\n");
	}
}

