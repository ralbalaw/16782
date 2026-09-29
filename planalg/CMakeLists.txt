cmake_minimum_required(VERSION 3.23)

project(planalg VERSION 0.1.0 LANGUAGES CXX)

add_library(planalg INTERFACE)
add_library(planalg::planalg ALIAS planalg)

target_compile_features(planalg INTERFACE cxx_std_20)

target_include_directories(
  planalg
  INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
