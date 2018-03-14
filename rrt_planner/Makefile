#########################################################
#														#
# RRT planner Makefile									#
# NOTE : Make cannot build ROS-related executables		#
# This Makefile can only be used:						#
#	- to build the 'rrtstar' standalone program			#
#	- as shortcut for some frequently used commands 	#
#														#
#########################################################

# 
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

warning:
	@echo "This makefile cannot build the package. Use catkin_make instead"

# Compiling the .c files -> .o
$(O_DIR)/%.o: $(SRC_DIR)/%.c $(DEPS)
	$(CXX) -c -o $@ $< $(CFLAGS)

# Compiling .cpp -> .o
$(O_DIR)/%.o: $(SRC_DIR)/%.cpp $(DEPS)
	$(CXX) -c -o $@ $< $(CFLAGS) -std=c++11

rrtstar: $(OBJ)
	$(CXX) -o $(BUILD_DIR)/$@ $^ $(CFLAGS)

.PHONY: clean doc mrproper viz debug rosTest cmd

cmd: 
	cd ~/catkin_ws &&\
	catkin_make --pkg rrt_planner  &&\
	#catkin_make --pkg rrt_planner -DCMAKE_BUILD_TYPE=Debug &&\
	cd -

rosTest: 
	roslaunch ./test.launch

# Run the standalone program
run: rrtstar
	./rrtstar
# Visualization of .sol files
viz:
	python viz/viz.py exp/basicExp-ros.sol

doc: 
	doxygen doxy.conf

clean:
	$(RM) build/*

mrproper: clean
	$(RM) rrtstar