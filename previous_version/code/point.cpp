/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<limits.h>

#include	"../headers/point.h"

Point::Point() {
	x = USHRT_MAX;
	y = USHRT_MAX;
	level = USHRT_MAX;
}

Point::Point(unsigned short px, unsigned short py, unsigned short pl) :
	x(px),
	y(py),
	level(pl) {
}

Point::Point(const Point &p) :
	x(p.x),
	y(p.y),
	level(p.level) {
}

Point::Point(const Point *p) :
	x(p->x),
	y(p->y),
	level(p->level) {
}

bool
Point::equals(const Point *p) const {
	if (x == p->x && y == p->y && level == p->level)
		return true;
	else
		return false;
}

bool
Point::equals(const Point &p) const {
	if (x == p.x && y == p.y && level == p.level)
		return true;
	else
		return false;
}

void
Point::print(FILE* out) const {
	fprintf(out, "%d %d - %d\n", x, y, level);
}
