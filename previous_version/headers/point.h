/* 
 *  
 */

#pragma once

#include	<stdlib.h>
#include	<stdio.h>

class Point {
 public:
	// Data
	unsigned short x;
	unsigned short y;
	unsigned short level;

	// Methods
	Point();
	Point(unsigned short px, unsigned short py, unsigned short pl = 0);
	Point(const Point &p);
	Point(const Point *p);

	bool equals(const Point *p) const;
	bool equals(const Point &p) const;
	void print(FILE* out) const;
};

