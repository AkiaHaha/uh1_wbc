cmake_minimum_required(VERSION 3.0.2)
project(unitree_wbc)


# Compile as C++14, which is supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# Eigen3
find_package(Eigen3 REQUIRED)

# RBDL
find_library(RBDL_LIBRARY NAMES rbdl PATHS /usr/local/lib)

# qpOASES
find_library(QPOASES_LIBRARY NAMES qpOASES PATHS /usr/local/lib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS EIGEN3
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  example/include
)

link_directories(/usr/local/lib)

# Add the RBDL include directory
include_directories(/usr/local/include/)

# Source files
aux_source_directory(src SRC_FILES)

# Add the library
add_library(${PROJECT_NAME} ${SRC_FILES})

# Link the libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARY}
  ${QPOASES_LIBRARY}
)

# Example
option(COMPILE_EXAMPLE "Compile the example (ON) or not (OFF)" ON)

if(COMPILE_EXAMPLE)
  # Webots controller library directories
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include_directories("/usr/local/webots/include/controller/c")
    include_directories("/usr/local/webots/include/controller/cpp")
    link_directories("/usr/local/webots/lib/controller")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    include_directories("/Applications/webots.app/include/controller/c")
    include_directories("/Applications/webots.app/include/controller/cpp")
    link_directories("/Applications/webots.app/lib/controller")
  endif()

  # Example source files
  file(GLOB EXAMPLE_SRC_FILES "example/src/*.cpp")

  # Setup the target executable for the example
  add_executable(uh1Controller ${EXAMPLE_SRC_FILES})

  # Link libraries for the example
  target_link_libraries(uh1Controller
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${RBDL_LIBRARY}
    ${QPOASES_LIBRARY}
    Controller  
    CppController 
  )

  # Custom command for the example
  add_custom_command(TARGET uh1Controller POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}/executables"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:uh1Controller>"
    "${CMAKE_SOURCE_DIR}/executables"
  )
endif(COMPILE_EXAMPLE)