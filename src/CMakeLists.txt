include(cmake/util.cmake)
common_settings()

add_subdirectory(3rdparty)
add_subdirectory(libs)
add_subdirectory(examples)
add_subdirectory(apps)
add_subdirectory(test)
