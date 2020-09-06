cmake_minimum_required(VERSION 3.10)
project(libutil VERSION 0.1 LANGUAGES CXX)

add_library_default(util)
target_link_libraries(util Magnum::Magnum)
