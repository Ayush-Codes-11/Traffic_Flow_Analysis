# Makefile – Traffic Simulation System
# Compatible with MinGW g++ on Windows (MSYS2) and Linux/Mac g++
#
# Usage:
#   make          → build the project
#   make run      → build + run
#   make clean    → delete build artefacts

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2

# Source files (Phase 1 + 2)
SRCS = src/graph.cpp src/dijkstra.cpp src/astar.cpp src/main.cpp

# Output binary
TARGET = traffic_sim.exe

# Default target: compile
all: $(TARGET)

$(TARGET): $(SRCS) src/graph.h
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)
	@echo "Build successful: $(TARGET)"

# Build and run
run: all
	./$(TARGET)

clean:
	del /Q $(TARGET) 2>nul || rm -f $(TARGET)

.PHONY: all run clean
