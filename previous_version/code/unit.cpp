/*
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<limits.h>

#include	"../headers/point.h"
#include	"../headers/unit.h"

Unit::Unit() {
	pos.x = USHRT_MAX;
	pos.y = USHRT_MAX;
	player = USHRT_MAX;
	range = USHRT_MAX;
	damage = USHRT_MAX;
	left = USHRT_MAX;
	right = USHRT_MAX;
	up = USHRT_MAX;
	down = USHRT_MAX;
}

Unit::Unit(unsigned short x, unsigned short y, unsigned short p, unsigned short r, unsigned short d, unsigned short l, unsigned short ri, unsigned short u, unsigned short dow) {
	pos.x = x;
	pos.y = y;
	player = p;
	range = r;
	damage = d;
	left = l;
	right = ri;
	up = u;
	down = dow;
}

Unit::Unit(const Unit &u) {
	pos.x = u.pos.x;
	pos.y = u.pos.y;
	player = u.player;
	range = u.range;
	damage = u.damage;
	left = u.left;
	right = u.right;
	down = u.down;
}

Unit::Unit(const Unit *u) {
	pos.x = u->pos.x;
	pos.y = u->pos.y;
	player = u->player;
	range = u->range;
	damage = u->damage;
	left = u->left;
	right = u->right;
	down = u->down;
}

bool
Unit::equals(const Unit *u) const {
	if (pos.x == u->pos.x && pos.y == u->pos.y && player == u->player && range == u->range && damage == u->damage && left == u->left && right == u->right && down == u->down)
		return true;
	else
		return false;
}

bool
Unit::equals(const Unit &u) const {
	if (pos.x == u.pos.x && pos.y == u.pos.y && player == u.player && range == u.range && damage == u.damage && left == u.left && right == u.right && down == u.down)
		return true;
	else
		return false;
}

void
Unit::print(FILE* out) const {
	fprintf(out, "%d %d - %d %d %d %d %d %d %d\n", pos.x, pos.y, player, range, damage, left, right, up, down);
}
