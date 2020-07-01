cmake_minimum_required(VERSION 3.13)
project(libenv VERSION 0.1 LANGUAGES CXX)

add_library_default(env)
target_link_libraries(env util)
