cmake_minimum_required(VERSION 3.23)

project(planalg VERSION 0.1.0 LANGUAGES CXX)

find_package(nlohmann_json REQUIRED)

add_library(planalg
  src/graph/graph.cpp
)
add_library(planalg::planalg ALIAS planalg)

target_compile_features(planalg PUBLIC cxx_std_20)

target_compile_definitions(
  planalg
  PRIVATE
  PLANNER_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

target_include_directories(
  planalg
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  planalg
  PRIVATE
  nlohmann_json::nlohmann_json
)