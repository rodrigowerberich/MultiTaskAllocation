CXX		  := g++
CXX_FLAGS := -Wall -Wextra -std=c++17 -ggdb

FOLDERS:= 	$(filter %/, $(wildcard */)) $(filter %/, $(wildcard */*/)) $(filter %/, $(wildcard */*/*/)) $(filter %/, $(wildcard */*/*/*/))
INCLUDE_FOLDERS= $(foreach dir, $(FOLDERS), -I$(dir))

BIN		:= bin
SRC		:= $(foreach dir, $(FOLDERS), $(wildcard $(dir)*.c)) $(foreach dir, $(FOLDERS), $(wildcard $(dir)*.cpp))
OBJ_C 	:= $(patsubst %.c, %.o, $(filter %.c,$(SRC)))
OBJ_CPP := $(patsubst %.cpp, %.o, $(filter %.cpp,$(SRC)))
INCLUDE	:= $(INCLUDE_FOLDERS) -I/usr/include/python2.7 
LIB		:= lib

LIBRARIES	:= -lm -lpython2.7 -lboost_iostreams -lboost_system -lboost_filesystem
EXECUTABLE	:= main


all: $(BIN)/$(EXECUTABLE)

run: clean all
	clear
	./$(BIN)/$(EXECUTABLE)

$(BIN)/$(EXECUTABLE): $(OBJ_C) $(OBJ_CPP)
	$(CXX) $(CXX_FLAGS) $(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

%.o : %.c
	@echo $@
	$(CXX) $(CXX_FLAGS) $(INCLUDE) -c $^ -o $@

%.o : %.cpp
	@echo $@
	$(CXX) $(CXX_FLAGS) $(INCLUDE) -c $^ -o $@

clean_objects:
	@rm $(OBJ_C)
	@rm $(OBJ_CPP)

clean: clean_objects
	@rm $(BIN)/*
	
