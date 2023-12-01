CC = g++
CFLAGS = -w -std=c++17
TARGETS = tests outputs/KdTree.o outputs/exceptions.o outputs/rrt.o outputs/helper.o outputs/tests.o outputs/planner.o

all: $(TARGETS) 

outputs/KdTree.o: Kd-Tree/KdTree.cpp 
	$(CC) $(CFLAGS) -c Kd-Tree/KdTree.cpp -o outputs/KdTree.o

outputs/exceptions.o: Kd-Tree/exceptions.cpp
	$(CC) $(CFLAGS) -c Kd-Tree/exceptions.cpp -o outputs/exceptions.o

outputs/rrt.o: RRT/RRT.cpp 
	$(CC) $(CFLAGS) -c RRT/RRT.cpp -o outputs/rrt.o 

outputs/helper.o: helper_funcs/helper.cpp
	$(CC) $(CFLAGS) -c helper_funcs/helper.cpp -o outputs/helper.o


outputs/planner.o: Planner/planner.cpp
	$(CC) $(CFLAGS) -c Planner/planner.cpp -o outputs/planner.o -lgmp

outputs/tests.o: tests.cpp 
	$(CC) $(CFLAGS) -c tests.cpp -o outputs/tests.o

tests: outputs/KdTree.o outputs/exceptions.o outputs/tests.o outputs/rrt.o outputs/helper.o outputs/planner.o
	$(CC) $(CFLAGS) outputs/KdTree.o outputs/exceptions.o outputs/tests.o outputs/rrt.o outputs/helper.o outputs/planner.o -o tests -lgmp

clean: 
	rm -rf *.o kd-tree/*.o rrt/*.o outputs/*.o