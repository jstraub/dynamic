project(dynamic)
cmake_minimum_required(VERSION 3.5)

if(POLICY CMP0012)
  cmake_policy(SET CMP0012 OLD)
endif()

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_COLOR_MAKEFILE ON)
IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug) # Release; debug; ...
ENDIF()

# add flags depending on build type
set(CMAKE_CXX_FLAGS_RELEASE "-g -Wall -Wno-deprecated -O2 -finline-functions -DNDEBUG -std=c++11 ")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wno-deprecated -std=c++11")

find_package(Pangolin REQUIRED)
find_package (Eigen3 3.2.93 REQUIRED)
#set(EIGEN3_INCLUDE_DIR "/home/jstraub/workspace/fun/eigen-eigen-5a0156e40feb/")
#message(${EIGEN3_INCLUDE_DIR})

file(GLOB SRCS
  "*.cpp"
  )

include_directories(${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIR} ${Pangolin_DIR}/include)

add_executable(dynamic
  ${SRCS}
  )
target_link_libraries(dynamic pangolin)


