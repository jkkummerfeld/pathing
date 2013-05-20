/*  Some methods for doing tricky manipulation on
 *  a node's internal state
 */

#include	<stdlib.h>
#include	<stdio.h>

#include	"../headers/node.h"
#include	"../headers/point.h"

Node::Node(const Node &n) :
	f(n.f),
	h(n.h),
	g(n.g),
	prev(n.prev),
	pos(n.pos.x, n.pos.y, n.pos.level) {
	state = 0;
	heap_pos = -1;
}

Node::Node(Node* n) :
	f(n->f),
	h(n->h),
	g(n->g),
	prev(n->prev),
	pos(n->pos.x, n->pos.y, n->pos.level) {
	state = 0;
	heap_pos = -1;
}

Node::Node(Point p) : pos(p.x, p.y, p.level) {
	f = -1;
	h = -1;
	g = -1;
	prev = NULL;
	state = 0;
	heap_pos = -1;
}

Node::Node(Point p, double pf, double pg, double ph, Node* pprev) :
	f(pf),
	h(ph),
	g(pg),
	prev(pprev),
	pos(p.x, p.y, p.level) {
	state = 0;
	heap_pos = -1;
}

void
Node::set_state(int val) {
	if ((state & val) == 0) {
		state += val;
	}
}

void
Node::unset_state(int val) {
	if ((state & val) != 0) {
		state -= val;
	}
}

int
Node::check_state(int val) const {
	return (state & val) != 0;
}

void
Node::print(FILE* out) const {
	fprintf(out, "(%d, %d - %d)\t", pos.x, pos.y, pos.level);
	fprintf(out, "%.1lf = %.1lf + %.1lf\t", f, g, h);
	fprintf(out, "%d %d %d %d %d\n", check_state(GOAL), check_state(START), check_state(WAITING), check_state(FRINGE), check_state(CLOSED));
}

