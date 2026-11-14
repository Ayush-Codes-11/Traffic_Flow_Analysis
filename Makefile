# Makefile – Traffic Simulation System
# Compatible with MinGW g++ on Windows (MSYS2) and Linux/Mac g++
#
# Usage:
#   make          → build the project
#   make run      → build + run
#   make clean    → delete build artefacts

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -static -static-libgcc -static-libstdc++

# Source files (Phase 1 + 2 + 3 + 4)
SRCS = src/graph.cpp src/dijkstra.cpp src/astar.cpp src/simulation.cpp src/assignment.cpp src/main.cpp

# Output binary and library
TARGET = traffic_sim.exe
DLL_TARGET = traffic_sim.dll

# Default target: compile
all: $(TARGET) $(DLL_TARGET)

$(TARGET): $(SRCS) src/graph.h
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)
	@echo "Build successful: $(TARGET)"

$(DLL_TARGET): $(SRCS) src/graph.h
	$(CXX) $(CXXFLAGS) -shared -o $(DLL_TARGET) $(SRCS)
	@echo "Build successful: $(DLL_TARGET)"

# Build and run
run: all
	./$(TARGET)

clean:
	del /Q $(TARGET) $(DLL_TARGET) 2>nul || rm -f $(TARGET) $(DLL_TARGET)

.PHONY: all run clean
