/* A binary min-heap for Nodes
 */

#pragma once

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include	"node.h"

class Heap {
 public:
// Methods
	Heap(int space);

	~Heap();

	/* Returns the node with lowest f
	 * or NULL if the heap is empty
	 */
	Node* pop();

	double cur_priority();

	void push(Node* node);

	void update(Node* node);

	bool is_empty();

 private:
// Data
	Node** heap;
	unsigned int len_heap;
	unsigned int heap_space;

// Methods
	void swap_down(unsigned int pos);
	void swap_up(unsigned int pos);
	void swap(unsigned int p0, unsigned int p1);
};

