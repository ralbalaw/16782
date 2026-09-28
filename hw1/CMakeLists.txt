cmake_minimum_required(VERSION 3.23)
project(planning_pset1)

# Add the main executable.
add_executable(run_test
        src/planner.cpp
        src/runtest.cpp
        src/utility.cpp)

target_compile_features(run_test PRIVATE cxx_std_20)

# Have the executable depend on the header files.
target_include_directories(run_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Define the absolute path to relevant directories.
target_compile_definitions(run_test PRIVATE MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/maps")
target_compile_definitions(run_test PRIVATE OUTPUT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/output")

target_link_libraries(run_test PRIVATE planalg::planalg)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
