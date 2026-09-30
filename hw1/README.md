# HW1

## Requirements

- CMake 3.23 or newer.
- C++20 or newer.
- Does not require any external libraries.

## Building

From root
```bash
cmake -B build
cmake --build build
```

## Running

From root
```bash
./build/run_test map9.
```

## File Guide

- **Planner:** `planner.h` declares the planner interface, and `src/planner.cpp` selects and runs the planner.
- **Runner:** `src/runtest.cpp` loads a map, runs the planner, and records the result.
- **Problem support:** `types.hpp` defines states and actions; `utility.hpp` and `src/utility.cpp` provide map and state operations.
- **Standard search:** `planalg/search/a_star_search.hpp` and `planalg/search/dijkstra_search.hpp` implement A* and Dijkstra search; `planalg/search/path.hpp` reconstructs paths.
- **Search support:** `planalg/search/types.hpp` defines OPEN, CLOSED, and other shared search types; `planalg/search/concepts.hpp` defines search interfaces; `planalg/search/utility.hpp` initializes search data; `planalg/search/time_limit.hpp` enforces planning time limits (used by `planalg/search/real_time_heuristic_search.hpp`).
- **Real-time search:** `planalg/search/real_time_heuristic_search.hpp` is the base class; `planalg/search/rtaa.hpp` and `planalg/search/lss_lrta.hpp` implement RTAA and LSS-LRTA.
- **Heuristics:** `planalg/search/heuristic/heuristic_manager.hpp` manages learned heuristics; `planalg/search/heuristic/moving_target_heuristic.hpp` and `planalg/search/heuristic/moving_target_grid_heuristic.hpp` provide moving-target heuristics; `planalg/search/heuristic/distance_metrics.hpp` provides distance functions.
- **Heuristic support:** `planalg/search/heuristic/types.hpp` defines shared heuristic types, and `planalg/search/heuristic/concepts.hpp` defines heuristic interfaces.
