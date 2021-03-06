cmake_minimum_required(VERSION 3.0)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES src/*.c)
file(GLOB CPP_SOURCES src/*.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})
set(WEBOTS_HOME "/usr/local/webots/")
# Link with the Webots controller library.
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}; ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp include ${Eigen_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES} yaml-cpp ${Boost_LIBRARIES})
get_target_property(OUT ${PROJECT} LINK_LIBRARIES)
message(STATUS ${OUT})

add_executable(test_program test_program.cpp src/supervisor_node.cpp src/camera_utils.cpp src/transform_utils.cpp)
target_link_libraries(test_program ${LIBRARIES} yaml-cpp ${Boost_LIBRARIES})


# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
