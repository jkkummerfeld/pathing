/*
 */

#pragma once

#include <vector>

#include "../headers/node.h"

using namespace std;

class Dependency_Map {
 public:
	unsigned short width;
	unsigned short height;
	unsigned short level;

	Dependency_Map(unsigned short l, unsigned short w, unsigned short h);

	~Dependency_Map();

	// 'waiter' cannot calculate its heuristic because the node it depends on,
	// 'unknown', has not been closed yet
	void add_dependency(Node* waiter, const Node& unknown);

	vector<Node*>* get_dependencies(const Node& n);

	void print(FILE* out);

 private:
	vector<Node*>** table;

	unsigned int get_pos(const Point& p);
};

