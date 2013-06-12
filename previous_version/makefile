FLAGS=-Wall -W -Wextra -g -O3
#FLAGS=-Wall -W -Wextra -g
CC=g++
search_objects=lib/manhattan.o lib/euclidean.o lib/octile.o lib/ucs.o lib/search.o lib/boundaries_non-blocking.o lib/boundaries_blocking.o lib/cucs_heuristic.o lib/corners_blocking.o lib/corners_non-blocking.o

all: dirs test bin/compare bin/check_accuracy bin/imba bin/benchmark bin/random_circular_threat bin/profile

test: bin/test_cost_map bin/test_heap bin/test_ucs bin/test_boundaries_blocking bin/test_path_map bin/test_cucs_heuristic bin/test_boundaries_non-blocking bin/test_corners_blocking bin/test_corners_non-blocking

dirs:
	if [ ! -d bin ] ; then mkdir bin ; fi
	if [ ! -d lib ] ; then mkdir lib ; fi

bin/check_accuracy: code/check_accuracy.cpp lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/path_map.o lib/dependency_map.o lib/unit.o $(search_objects)
	$(CC) $(FLAGS) $^ -o $@

bin/profile: code/profile.cpp lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/unit.o lib/path_map.o lib/dependency_map.o lib/unit.o $(search_objects)
	$(CC) $(FLAGS) $^ -o $@

bin/compare: code/compare.cpp lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/unit.o lib/path_map.o lib/dependency_map.o lib/unit.o $(search_objects)
	$(CC) $(FLAGS) $^ -o $@

bin/random_circular_threat: code/random_circular_threat.cpp lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/path_map.o lib/unit.o lib/dependency_map.o $(search_objects)
	$(CC) $(FLAGS) $^ -o $@

bin/imba: code/imba_data_test.cpp lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/path_map.o lib/dependency_map.o lib/unit.o $(search_objects)
	$(CC) $(FLAGS) $^ -o $@

bin/benchmark: code/benchmark_test.cpp lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/path_map.o lib/dependency_map.o lib/unit.o $(search_objects)
	$(CC) $(FLAGS) $^ -o $@

bin/test_ucs: code/ucs_test.cpp lib/ucs.o lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/search.o lib/path_map.o lib/unit.o
	$(CC) $(FLAGS) $^ -o $@

lib/ucs.o: code/ucs.cpp headers/ucs.h
	$(CC) $(FLAGS) code/ucs.cpp -c -o $@

lib/manhattan.o: code/manhattan.cpp headers/manhattan.h
	$(CC) $(FLAGS) code/manhattan.cpp -c -o $@

lib/euclidean.o: code/euclidean.cpp headers/euclidean.h
	$(CC) $(FLAGS) code/euclidean.cpp -c -o $@

lib/octile.o: code/octile.cpp headers/octile.h
	$(CC) $(FLAGS) code/octile.cpp -c -o $@

bin/test_cucs_heuristic: code/cucs_heuristic_test.cpp lib/cucs_heuristic.o lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/search.o lib/path_map.o lib/unit.o
	$(CC) $(FLAGS) $^ -o $@

lib/cucs_heuristic.o: code/cucs_heuristic.cpp headers/cucs_heuristic.h
	$(CC) $(FLAGS) code/cucs_heuristic.cpp -c -o $@

bin/test_boundaries_non-blocking: code/boundaries_non-blocking_test.cpp lib/boundaries_non-blocking.o lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/search.o lib/path_map.o lib/dependency_map.o lib/unit.o
	$(CC) $(FLAGS) $^ -o $@

lib/boundaries_non-blocking.o: code/boundaries_non-blocking.cpp headers/boundaries_non-blocking.h
	$(CC) $(FLAGS) code/boundaries_non-blocking.cpp -c -o $@

bin/test_boundaries_blocking: code/boundaries_blocking_test.cpp lib/boundaries_blocking.o lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/search.o lib/path_map.o lib/unit.o
	$(CC) $(FLAGS) $^ -o $@

lib/boundaries_blocking.o: code/boundaries_blocking.cpp headers/boundaries_blocking.h
	$(CC) $(FLAGS) code/boundaries_blocking.cpp -c -o $@

bin/test_corners_non-blocking: code/corners_non-blocking_test.cpp lib/corners_non-blocking.o lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/search.o lib/path_map.o lib/dependency_map.o lib/unit.o
	$(CC) $(FLAGS) $^ -o $@

lib/corners_non-blocking.o: code/corners_non-blocking.cpp headers/corners_non-blocking.h
	$(CC) $(FLAGS) code/corners_non-blocking.cpp -c -o $@

bin/test_corners_blocking: code/corners_blocking_test.cpp lib/corners_blocking.o lib/heap.o lib/cost_map.o lib/node.o lib/path.o lib/point.o lib/search.o lib/path_map.o lib/unit.o
	$(CC) $(FLAGS) $^ -o $@

lib/corners_blocking.o: code/corners_blocking.cpp headers/corners_blocking.h
	$(CC) $(FLAGS) code/corners_blocking.cpp -c -o $@

lib/search.o: code/search.cpp headers/search.h
	$(CC) $(FLAGS) code/search.cpp -c -o $@

bin/test_cost_map: code/cost_map_test.cpp lib/cost_map.o lib/point.o lib/unit.o
	$(CC) $(FLAGS) $^ -o $@

lib/cost_map.o: code/cost_map.cpp headers/cost_map.h
	$(CC) $(FLAGS) code/cost_map.cpp -c -o $@

bin/test_path_map: code/path_map_test.cpp lib/path_map.o lib/node.o lib/point.o
	$(CC) $(FLAGS) $^ -o $@

lib/path_map.o: code/path_map.cpp headers/path_map.h
	$(CC) $(FLAGS) code/path_map.cpp -c -o $@

lib/dependency_map.o: code/dependency_map.cpp headers/dependency_map.h
	$(CC) $(FLAGS) code/dependency_map.cpp -c -o $@

bin/test_heap: code/heap_test.cpp lib/heap.o lib/node.o lib/point.o
	$(CC) $(FLAGS) $^ -o $@

lib/heap.o: code/heap.cpp headers/heap.h
	$(CC) $(FLAGS) code/heap.cpp -c -o $@

lib/path.o: code/path.cpp headers/path.h
	$(CC) $(FLAGS) code/path.cpp -c -o $@

lib/node.o: code/node.cpp headers/node.h
	$(CC) $(FLAGS) code/node.cpp -c -o $@

lib/unit.o: code/unit.cpp headers/unit.h
	$(CC) $(FLAGS) code/unit.cpp -c -o $@

lib/point.o: code/point.cpp headers/point.h
	$(CC) $(FLAGS) code/point.cpp -c -o $@

clean:
	rm lib/*
	rm -rf bin/*

