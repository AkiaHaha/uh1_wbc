cmake_minimum_required( VERSION 3.0 )

# Setup the project
set(CMAKE_BUILD_TYPE Debug)  # 添加这行确保生成调试信息 //Daniel
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

configure_file (
"${PROJECT_SOURCE_DIR}/include/taichi/config.h.in"
"${PROJECT_BINARY_DIR}/include/taichi/config.h"
)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

option (USE_ERROR
       "The Error Message display as a warning (OFF) or catch it yourself (ON)" ON)
option (COMPILE_EXAMPLE
       "Compile the example (ON) or not (OFF)" ON)

if(USE_ERROR)
    message(STATUS "USE_ERROR = ${BoldGreen}ON${ColourReset}")
else()
    message(STATUS "USE_ERROR = ${BoldBlue}OFF${ColourReset}")
endif(USE_ERROR)
if(COMPILE_EXAMPLE)
    message(STATUS "COMPILE_EXAMPLE = ${BoldGreen}ON${ColourReset}")
else()
message(STATUS "COMPILE_EXAMPLE = ${BoldBlue}OFF${ColourReset}")
endif(COMPILE_EXAMPLE)

if(USE_ERROR)
    message(STATUS "${Red}Catch and handle the error message yourself, if not fatal problem may occur!${ColourReset}")
else()
    message("${Red}The Error Message displays as a warning, it won't kill the program!${ColourReset}")
endif(USE_ERROR)

if(COMPILE_EXAMPLE)
    message(STATUS "${Red}The example code will be compiled!${ColourReset}")
else()
    message("${Red}The example code will not be compiled!${ColourReset}")
endif(COMPILE_EXAMPLE)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2 -Wall")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Search for the Eigen3 include directory and library
find_package (Eigen3 REQUIRED)

if(Eigen3)
    message(STATUS "EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
	message(STATUS "EIGEN3_LIBRARIES=${EIGEN3_LIBRARIES}")	
endif()

include_directories (${EIGEN3_INCLUDE_DIR})

# Add the RBDL library
include_directories(/usr/local/include/)
add_library(rbdl STATIC IMPORTED)
set_property(TARGET rbdl PROPERTY IMPORTED_LOCATION /usr/local/lib/librbdl.so)

# Add the qpOASES library
add_library(qpOASES STATIC IMPORTED)
set_property(TARGET qpOASES PROPERTY IMPORTED_LOCATION /usr/local/lib/libqpOASES.a)

include_directories(${PROJECT_SOURCE_DIR}/include/taichi)
aux_source_directory(${PROJECT_SOURCE_DIR}/src TAICHI_SRC_DIR)

add_library( taichi STATIC ${TAICHI_SRC_DIR} )

install(TARGETS taichi ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/taichi)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/taichi DESTINATION ${PROJECT_SOURCE_DIR}/build/taichi)


if(COMPILE_EXAMPLE)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        # Link the Webots controller library.
        include_directories("/usr/local/webots/include/controller/c")
        include_directories("/usr/local/webots/include/controller/cpp")
        link_directories("/usr/local/webots/lib/controller")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        # Link the Webots controller library.
        include_directories("/Applications/webots.app/include/controller/c")
        include_directories("/Applications/webots.app/include/controller/cpp")
        link_directories("/Applications/webots.app/lib/controller")
    endif()
    set (LIBRARIES
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
    )


    # Compile examples
    include_directories(${PROJECT_SOURCE_DIR}/example/Biped/include)
    aux_source_directory(${PROJECT_SOURCE_DIR}/example/Biped/src Biped_DIR)

    # Setup the target executable
    add_executable("BipedController" ${Biped_DIR})
    target_link_libraries("BipedController" ${LIBRARIES} taichi rbdl qpOASES)
    add_custom_command(TARGET "BipedController" POST_BUILD COMMAND ${CMAKE_COMMAND} -E
                        make_directory "${CMAKE_SOURCE_DIR}/example/Biped/WebotsProject/controllers/BipedController/")
    add_custom_command(TARGET "BipedController" POST_BUILD COMMAND ${CMAKE_COMMAND} -E
                        copy ${CMAKE_BINARY_DIR}/"BipedController"
                        ${CMAKE_SOURCE_DIR}/example/Biped/WebotsProject/controllers/BipedController/"BipedController")


endif(COMPILE_EXAMPLE)



