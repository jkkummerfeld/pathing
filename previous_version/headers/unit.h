/* 
 *  
 */

#pragma once

#include "point.h"
///#include	<stdlib.h>

class Unit {
 public:
	// Data
	Point pos;
	unsigned short player;
	unsigned short range;
	unsigned short damage;
	unsigned short left;
	unsigned short right;
	unsigned short up;
	unsigned short down;

	// Methods
	Unit();
	Unit(unsigned short x, unsigned short y, unsigned short player, unsigned short range, unsigned short damage, unsigned short left, unsigned short right, unsigned short up, unsigned short down);
	Unit(const Unit &u);
	Unit(const Unit *u);

	bool equals(const Unit *u) const;
	bool equals(const Unit &u) const;
	void print(FILE* out) const;
};

