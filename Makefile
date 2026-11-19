CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
LIB      := lib/protocol.cpp lib/noisy_link.cpp

.PHONY: all run test clean test-all test-py test-integration test-blackbox test-replay sitl sitl-test sitl-deps mav info replay

all: build/drone_sim build/gcs_bridge build/mavlink_bridge build/blackbox

build/drone_sim: src/drone_sim.cpp $(LIB) | build
	$(CXX) $(CXXFLAGS) -Ilib -pthread $^ -o $@

build/gcs_bridge: src/gcs_bridge.cpp lib/protocol.cpp | build
	$(CXX) $(CXXFLAGS) -Ilib $^ -o $@

# MAVLink v1/v2 (UDP / serial / file) -> uavproto packets on stdout.
build/mavlink_bridge: src/mavlink_bridge.cpp lib/protocol.cpp | build
	$(CXX) $(CXXFLAGS) -Ilib $^ -o $@

# Black box: record / replay / info for the byte stream in front of the parser.
build/blackbox: src/blackbox.cpp | build
	$(CXX) $(CXXFLAGS) $^ -o $@

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

# --- MAVLink bridge and black box ------------------------------------------
# mavlink_bridge (file / stdin / UDP, v1 + v2, bad CRC, false frame starts) and blackbox.
test-integration: build/mavlink_bridge build/blackbox
	python3 tests/test_integration.py ./build/mavlink_bridge ./build/blackbox

# Record -> replay round trip: passthrough, --speed, truncated log, --loop.
test-blackbox: build/blackbox
	python3 tools/test_blackbox.py

# Live vs replay at 10 % noise: same bytes, same JSON, same parser statistics.
test-replay: all
	python3 tools/test_replay_equivalence.py --noise 0.10

# Everything that needs no vehicle and no hardware.
test-all: test-py test-integration test-blackbox test-replay

# Dashboard fed by build/mavlink_bridge on UDP 14550 (SITL or a real autopilot).
mav: all
	python3 server/server.py --source mav

# Summary of a recording: make info LOG=flights/demo.ubx
LOG ?= flights/demo.ubx
info: build/blackbox
	./build/blackbox info $(LOG)

# Dashboard replaying a recording: make replay LOG=flights/demo.ubx SPEED=4
SPEED ?= 1
replay: all
	python3 server/server.py --replay $(LOG) --speed $(SPEED)
