/*
 */

#include	<math.h>

#include "../headers/cost_map.h"
#include "../headers/node.h"

#include "../headers/path_map.h"

Path_Map::Path_Map(unsigned short l, unsigned short w, unsigned short h, unsigned int nodes) : width(w), height(h), level(l) {
	len_table = nodes * 10;
///	len_table = width * height;
	len_table = 50000017;
	table = (Node**) calloc(len_table, sizeof(Node*));
}

Path_Map::~Path_Map() {
	for (unsigned int i = 0; i < len_table; i++)
		delete table[i];
	free(table);
}

unsigned int
Path_Map::hash(const Point &p) {
///	unsigned int ans = 0;
///	ans += p.x;
///	ans *= 2003;
///	ans += p.y;
	return p.x * 1000003 + p.y * 1000081;
///	return p.x * height + p.y;
}

unsigned int
Path_Map::get_pos(const Point &p) {
	if (p.level != level) {
		fprintf(stderr, "Point requested from wrong path map.\n");
		return -1;
	}
	unsigned int pos = hash(p) % len_table;
	Node* node = NULL;
	unsigned int opos = pos;
	int steps = 0;
	while (true) {
		node = table[pos];
		if (node == NULL) {
			if (steps > 5)
				printf("%d steps\n", steps);
			return pos;
		}
		else if (node->pos.equals(p)) {
			if (steps > 5)
				printf("%d steps\n", steps);
			return pos;
		}
		pos++;
		if (pos == len_table)
			pos = 0;
		if (pos == opos)
			break;
		steps += 1;
	}
	fprintf(stderr, "Ran out of space in a path map hash table. Current space %d\n", len_table);
	return -1;
}

Node*
Path_Map::get_node(const Point &p) {
	unsigned int pos = get_pos(p);
	Node* node = table[pos];
	if (node == NULL) {
		node = new Node(p);
		node->f = -1;
		node->g = -1;
		node->heap_pos = -1;
		node->prev = NULL;
		table[pos] = node;
	}
	return node;
}

void
Path_Map::print(FILE* out) {
	Point p(0, 0, level);
	for (unsigned int j = 0; j <= height; j++) {
		for (unsigned int i = 0; i <= width; i++) {
			p.x = i;
			p.y = j;
			Node* node = table[get_pos(&p)];
			if (node == NULL || node->g == HUGE_VAL || node->g < 0)
				fprintf(out, "      ");
			else if (node->check_state(WAITING) || node->check_state(FRINGE))
				fprintf(out, "%4.1f_ ", node->g);
			else
				fprintf(out, "%4.1f  ", node->g);
		}
		fprintf(out, "\n");
	}
}

