cmake_minimum_required(VERSION 3.5)
project(MapTools)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to release")
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

# output paths settings
set(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")

# work out the postfix; required where we use OUTPUT_NAME
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(EXE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
    set(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
    add_definitions("-DDEBUG")
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(UNIX)
    SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-pthread")
endif(UNIX)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
MESSAGE(STATUS "Executable Output Path : ${EXECUTABLE_OUTPUT_PATH}")
MESSAGE(STATUS "Library Output Path : ${LIBRARY_OUTPUT_PATH}")

#find_package(lcm REQUIRED HINTS ${PROJECT_SOURCE_DIR}/3rdParty/lcm/build)
#MESSAGE(STATUS "Lcm Found : ${lcm_FOUND}")
#MESSAGE(STATUS "Lcm Include Dirs : ${lcm_INCLUDE_DIRS}")
#MESSAGE(STATUS "Lcm Libraries : ${LCM_LIBRARIES}")

include_directories(src)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(3rdParty)

add_subdirectory(src)
add_subdirectory(app)
add_subdirectory(test2)

option(UseTest "Compile Test" OFF)
MESSAGE(STATUS "Use Test or not : ${UseTest}")
if(UseTest)
    add_subdirectory(Test)
endif()

