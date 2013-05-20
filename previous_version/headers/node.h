/* A node for use in the pq and in the map
 * (to save memory)
 */

#pragma once

#include "point.h"

/* States */
#define	FRINGE	1
#define	CLOSED	2
#define	START	4
#define	GOAL	8
#define	WAITING	16

class Node {
 public:
// Data
	/* Search related info */
	double f;
	double h;
	double g;
	Node* prev;
	short state;

	/* Map info */
	Point pos;

	/* PQ info */
	int heap_pos;

// Methods
	Node(const Node &n);
	Node(Node* n);
	Node(Point p);
	Node(Point p, double pf, double pg, double ph, Node* pprev);

	void set_state(int val);
	void unset_state(int val);
	int check_state(int val) const;

	void print(FILE* out) const;
};

