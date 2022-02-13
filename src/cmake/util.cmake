set(CURRENT_DIR "${CMAKE_CURRENT_LIST_DIR}")

macro(set_compiler_flags)
  if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Za")
    message(STATUS "Disable Microsoft language extensions: ${CMAKE_CXX_FLAGS}")
  endif()

  if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    message(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
  endif()

  if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Ot")
    message(STATUS "Added optimization flags CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
  endif()

  if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  else()
    add_compile_options(-Wall -Wextra -Wdelete-non-virtual-dtor)
  endif()

  add_definitions(-D_SCL_SECURE_NO_WARNINGS)  # VS annoying warnings
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)  # VS annoying warnings
  add_definitions(-D_USE_MATH_DEFINES)  # to enable stuff like M_PI
endmacro()

macro(find_modules)
  find_package(OpenCV REQUIRED)
  include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
endmacro()

macro(common_settings)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  set_compiler_flags()

  find_modules()
endmacro()

macro(collect_sources_default name)
  file(GLOB SOURCES src/*.c src/*.cpp)
  file(GLOB HEADERS include/${name}/*.h include/${name}/*.hpp)
  message(STATUS "${name} sources ${SOURCES}")
  message(STATUS "${name} headers ${HEADERS}")
endmacro()

macro(set_default_properties target folder_name)
  set_target_properties(${target} PROPERTIES FOLDER ${folder_name})
endmacro()

macro(add_library_default name)
  collect_sources_default(${name})
  add_library(${name} STATIC ${SOURCES} ${HEADERS})
  include_directories(include)
  set_default_properties(${name} "libs")
  target_include_directories(${name} PUBLIC include)
endmacro()

macro(add_app_default name src)
  message(STATUS "APP ${name} sources ${src}")
  add_executable(${name} ${src})
  set_default_properties(${name} "apps")
endmacro()

macro(add_test_default name)
  collect_sources_default(${name})
  add_executable(${name} ${SOURCES} ${HEADERS})
  set_default_properties(${name} "tests")
endmacro()
