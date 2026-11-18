CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
LIB      := lib/protocol.cpp lib/noisy_link.cpp

.PHONY: all run test clean test-py sitl sitl-test sitl-deps

all: build/drone_sim build/gcs_bridge

build/drone_sim: src/drone_sim.cpp $(LIB) | build
	$(CXX) $(CXXFLAGS) -Ilib -pthread $^ -o $@

build/gcs_bridge: src/gcs_bridge.cpp lib/protocol.cpp | build
	$(CXX) $(CXXFLAGS) -Ilib $^ -o $@

build:
	mkdir -p build

# Quick check without a browser: 1 second of flight, last 3 JSON lines.
test: all
	./build/drone_sim --noise 0.05 --rate 200 --duration 1 | ./build/gcs_bridge | tail -3

run: all
	python3 server/server.py

clean:
	rm -rf build

# --- ArduPilot SITL over MAVLink -------------------------------------------
sitl-deps:
	python3 -m pip install --user pymavlink

# Encoder matches protocol.hpp (no vehicle needed).
test-py:
	python3 tools/test_mavlink_source.py

# 10 s from a running SITL, last 5 JSON lines (start sim_vehicle.py first).
sitl-test: build/gcs_bridge
	python3 tools/mavlink_source.py --duration 10 | ./build/gcs_bridge | tail -5

# Dashboard fed by SITL, with the scripted takeoff + square flight.
sitl: all
	python3 server/server.py --source sitl --demo
