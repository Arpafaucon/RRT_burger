CXX=g++
CC=gcc
# define needed headers files
_DEPS=kdtree.h rrts.h system.h system_single_integrator.h
# define needed C/Cpp files
_OBJ=kdtree.o rrts_standalone.o system_single_integrator.o
# Where to find all that
I_DIR=include/rrt_planner
O_DIR=build
SRC_DIR=src
#automatically forms corrects paths
DEPS=$(patsubst %, $(I_DIR)/%, $(_DEPS))
OBJ= $(patsubst %, $(O_DIR)/%, $(_OBJ))

CFLAGS=-g -I $(I_DIR)
BUILD_DIR=.

# A obj/.o file depends on src/.c and src/.files
# he is created by compiled the coreesponding .c
$(O_DIR)/%.o: $(SRC_DIR)/%.c $(DEPS)
	$(CXX) -c -o $@ $< $(CFLAGS)

# A obj/.o file depends on src/.c and src/.files
# he is created by compiled the coreesponding .c
$(O_DIR)/%.o: $(SRC_DIR)/%.cpp $(DEPS)
	$(CXX) -c -o $@ $< $(CFLAGS) -std=c++11

rrtstar: $(OBJ)
	echo $(I_DIR)
	$(CXX) -o $(BUILD_DIR)/$@ $^ $(CFLAGS)

cmd: 
	cd ~/catkin_ws &&\
	catkin_make --pkg rrt_planner -DCMAKE_BUILD_TYPE=Debug &&\
	cd -

rosTest: 
	roslaunch ./test.launch

run: rrtstar
	./rrtstar

debug:
	(roslaunch simulation/tworld.launch)&
	(roslaunch simulation/navigation.launch)

.PHONY: clean doc mrproper

clean:
	$(RM) build/*

doc: 
	doxygen doxy/doxy.conf

mrproper: clean
	$(RM) rrtstar