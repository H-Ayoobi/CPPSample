cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Find PCL 1.7
find_package(PCL 1.8 REQUIRED)
if (NOT PCL_FOUND)
	MESSAGE("PCL 1.8 not found.\n")
endif (NOT PCL_FOUND)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


# # hybrid nodelet library feature extraction
rosbuild_add_library (simulated_teacher_functionality src/simulated_teacher_functionality.cpp)
SET_TARGET_PROPERTIES(simulated_teacher_functionality PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(simulated_teacher_functionality ${PCL_LIBRARIES})
target_link_libraries(simulated_teacher_functionality perception_db)
target_link_libraries(simulated_teacher_functionality boost_system)


rosbuild_add_executable(simulated_teacher src/simulated_teacher.cpp)
SET_TARGET_PROPERTIES(simulated_teacher PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(simulated_teacher ${PCL_LIBRARIES})
target_link_libraries(simulated_teacher perception_db)
target_link_libraries(simulated_teacher boost_system)
target_link_libraries(simulated_teacher spin_image)
target_link_libraries(simulated_teacher simulated_teacher_functionality)

