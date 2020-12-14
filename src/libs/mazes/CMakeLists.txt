cmake_minimum_required(VERSION 3.10)
project(libmazes VERSION 0.1 LANGUAGES CXX)

add_library_default(mazes)
target_link_libraries(mazes PUBLIC util)
