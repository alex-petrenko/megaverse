cmake_minimum_required(VERSION 3.10)
project(libscenarios VERSION 0.1 LANGUAGES CXX)

add_library_default(scenarios)
target_link_libraries(scenarios PUBLIC env util)
