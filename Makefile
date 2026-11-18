CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
LIB      := lib/protocol.cpp lib/noisy_link.cpp

.PHONY: all run test clean

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
