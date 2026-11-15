# Makefile – Traffic Simulation System
# Compatible with GCC/Clang on Linux, macOS, and Windows (MinGW)
#
# Usage:
#   make          → build the project
#   make run      → build + run
#   make clean    → delete build artifacts

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -pthread

# Source files (Phase 1 through Phase 5)
SRCS = src/graph.cpp src/dijkstra.cpp src/astar.cpp src/simulation.cpp src/assignment.cpp src/alerts.cpp src/incidents.cpp src/sensor_feed.cpp src/main.cpp

# Output binary
TARGET = traffic_sim

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)
	@echo "Build successful: $(TARGET)"

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe *.o *.dll 2>/dev/null || del /Q $(TARGET) $(TARGET).exe *.o *.dll 2>nul || true

.PHONY: all run clean
