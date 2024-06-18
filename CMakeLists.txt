cmake_minimum_required(VERSION 3.14)
project(dynablox_benchmark)

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680#issuecomment-700642602
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#############
# PACKAGES  #
#############
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(voxblox)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_BINARY_DIR}/voxblox")
message("-------------------CMAKE_PREFIX_PATH----------------------")
cmake_print_variables(CMAKE_PREFIX_PATH)

find_package(PCL REQUIRED)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(voxblox REQUIRED)
set(LOG_LIB glog::glog gflags)
set(FUN_LIB ${PCL_LIBRARIES} voxblox::voxblox Eigen3::Eigen yaml-cpp)

#############
# LIBRARIES #
#############
include_directories(${CMAKE_SOURCE_DIR}/build)
include_directories(include)

add_library(dm_lib SHARED 
            src/dynablox.cpp 
            src/clustering.cpp 
            src/TsdfMapper.cpp)
add_dependencies(dm_lib voxblox)
target_link_libraries(dm_lib ${LOG_LIB} yaml-cpp ${FUN_LIB})

add_executable(dynablox_run dynablox_pcd.cpp)
target_link_libraries(dynablox_run dm_lib)