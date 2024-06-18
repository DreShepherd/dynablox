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

if(NOT TARGET PCL)
    find_package(PCL REQUIRED)
endif()
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
if (NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 REQUIRED)
endif()
find_package(yaml-cpp REQUIRED)
if (NOT TARGET voxblox)
    find_package(voxblox REQUIRED)
endif()
set(LOG_LIB glog::glog gflags)
set(FUN_LIB ${PCL_LIBRARIES} voxblox Eigen3::Eigen yaml-cpp)

#############
# LIBRARIES #
#############
add_library(dm_lib SHARED 
            src/dynablox.cpp 
            src/clustering.cpp 
            src/TsdfMapper.cpp)
add_dependencies(dm_lib voxblox)
target_link_libraries(dm_lib ${LOG_LIB} ${FUN_LIB})
target_include_directories(dm_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/dm_lib>
)

add_executable(dynablox_run dynablox_pcd.cpp)
target_link_libraries(dynablox_run dm_lib)