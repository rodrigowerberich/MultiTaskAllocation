CXX		  := g++
CXX_FLAGS := -Wall -Wextra -std=c++17 -ggdb

FOLDERS:= 	$(filter %/, $(wildcard */)) $(filter %/, $(wildcard */*/)) $(filter %/, $(wildcard */*/*/)) $(filter %/, $(wildcard */*/*/*/))
INCLUDE_FOLDERS= $(foreach dir, $(FOLDERS), -I$(dir))

BIN		:= bin
SRC		:= $(foreach dir, $(FOLDERS), $(wildcard $(dir)*.c)) $(foreach dir, $(FOLDERS), $(wildcard $(dir)*.cpp))
INCLUDE	:= $(INCLUDE_FOLDERS) -I/usr/include/python2.7 
LIB		:= lib

LIBRARIES	:= -lm -lpython2.7 -lboost_iostreams -lboost_system -lboost_filesystem
EXECUTABLE	:= main


all: $(BIN)/$(EXECUTABLE)

run: clean all
	clear
	./$(BIN)/$(EXECUTABLE)

$(BIN)/$(EXECUTABLE): $(SRC)
	$(CXX) $(CXX_FLAGS) $(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

clean:
	-rm $(BIN)/*