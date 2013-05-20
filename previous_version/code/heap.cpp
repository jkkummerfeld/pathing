/* A binary min-heap for Nodes
 *
 * Uses an array, expanding as needed
 */

#include	<stdlib.h>
#include	<stdio.h>
#include	<math.h>

#include	"../headers/node.h"
#include	"../headers/heap.h"

void
Heap::swap(unsigned int p0, unsigned int p1) {
	Node* tmp = heap[p1];
	heap[p1] = heap[p0];
	heap[p0] = tmp;

	heap[p1]->heap_pos = p1;
	heap[p0]->heap_pos = p0;
}

void
Heap::swap_down(unsigned int pos) {
	unsigned int child1 = pos * 2 + 1;
	unsigned int child2 = pos * 2 + 2;
	if (child1 < len_heap) {
		if (child2 < len_heap) {
			// compare both, take smaller and recurse
			if ((heap[pos])->f > (heap[child1])->f &&
				(heap[child1])->f <= (heap[child2])->f) {
				swap(pos, child1);
				swap_down(child1);
			} else if ((heap[pos])->f > (heap[child2])->f &&
				(heap[child2])->f <= (heap[child1])->f) {
				swap(pos, child2);
				swap_down(child2);
			}
		} else {
			// compare with the one that is present
			if ((heap[pos])->f > (heap[child1])->f) {
				swap(pos, child1);
			}
		}
	}
}

void
Heap::swap_up(unsigned int pos) {
	if (pos > 0) {
		// This heap is indexed from 0, so we have:
		// 0 is the parent of 1 and 2
		// 1 is the parent of 3 and 4
		// 2 is the parent of 5 and 6
		// etc
		unsigned int parent = (pos - 1) / 2;
		if ((heap[pos])->f < (heap[parent])->f) {
			swap(pos, parent);
			swap_up(parent);
		}
	}
}

/* Public methods
 */

Heap::Heap(int space=1024) {
	len_heap = 0;
	heap_space = space;
	heap = (Node**) malloc(sizeof(Node*) * heap_space);
}

Heap::~Heap() {
	free(heap);
}

Node*
Heap::pop() {
	if (len_heap == 0)
		return NULL;
	Node* ans = heap[0];
	swap(0, len_heap - 1);
	len_heap--;
	swap_down(0);
	ans->unset_state(FRINGE);
	return ans;
}

double
Heap::cur_priority() {
	if (len_heap > 0)
		return heap[0]->f;
	else
		return HUGE_VAL;
}

void
Heap::push(Node* node) {
	if (len_heap == heap_space) {
		heap_space *= 2;
		Node** tmp = (Node**) realloc(heap, heap_space * sizeof(Node*));
		if (tmp == NULL)
			fprintf(stderr, "Unable to allocate more space for a heap/pqueue\n");
		else
			heap = tmp;
	}
	heap[len_heap] = node;
	heap[len_heap]->heap_pos = len_heap;
	heap[len_heap]->set_state(FRINGE);
	len_heap++;
	swap_up(len_heap - 1);
}

void
Heap::update(Node* node) {
	swap_up(node->heap_pos);
	swap_down(node->heap_pos);
}

bool
Heap::is_empty() {
	return (len_heap == 0);
}

