cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# include opencv
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(src/API_Turtlebot.hpp )
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


rosbuild_add_executable(testopencv ${HDRS} ./src/testopencv.cpp)
target_link_libraries(testopencv ${OpenCV_LIBRARIES})

find_package(VISP REQUIRED)
if(VISP_FOUND)
  add_definitions(${VISP_DEFINITIONS})
  include_directories(${VISP_INCLUDE_DIRS})
  link_directories(${VISP_LIBRARY_DIRS})
  link_libraries(${VISP_LIBRARIES})
endif(VISP_FOUND)

rosbuild_add_executable(visioncontrol src/vision-control-beta.cpp)
target_link_libraries(visioncontrol ${OpenCV_LIBRARIES})



