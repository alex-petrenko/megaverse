message(STATUS "FLAGS: ${CMAKE_CXX_FLAGS}")

add_subdirectory(googletest-1.10.0)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_subdirectory(glad)
