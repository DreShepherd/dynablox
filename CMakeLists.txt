cmake_minimum_required(VERSION 3.14)
project(dynablox)

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
add_library(dyna_lib
    src/dynablox.cpp 
    src/clustering.cpp 
    src/TsdfMapper.cpp)
target_link_libraries(dyna_lib ${LOG_LIB} ${FUN_LIB})
target_include_directories(dyna_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/dyna_lib>
)
install(TARGETS dyna_lib
	EXPORT dyna_libTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
INSTALL(EXPORT dyna_libTargets
	FILE "dyna_libTargets.cmake"
	NAMESPACE dyna_lib::
	DESTINATION lib/cmake/dyna_lib
)
include(CMakePackageConfigHelpers)
configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/dyna_libConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/dyna_libConfig.cmake"
	INSTALL_DESTINATION lib/cmake/dyna_lib
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/dyna_libConfigVersion.cmake"
	VERSION 0.0.1
	COMPATIBILITY SameMajorVersion
)

INSTALL(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/dyna_libConfig.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/dyna_libConfigVersion.cmake"
	DESTINATION lib/cmake/dyna_lib
)
install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)

add_executable(dynablox_run dynablox_pcd.cpp)
target_link_libraries(dynablox_run dyna_lib)