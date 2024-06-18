cmake_minimum_required(VERSION 3.17)
project(voxblox
    VERSION 1.0.0
    DESCRIPTION "Voxblox Library"
    LANGUAGES CXX
)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT TARGET Boost)
    find_package(Boost REQUIRED COMPONENTS system)
endif()
find_package(Eigen3 REQUIRED CONFIG)
if (NOT TARGET Eigen3::Eigen)
    # workaround: the official Eigen uses the Eigen3::Eigen target name but the aimotive fork use ::Eigen3
    set_target_properties(::Eigen3 PROPERTIES IMPORTED_GLOBAL TRUE)
    add_library(Eigen3::Eigen ALIAS ::Eigen3)
endif()
find_package(OpenMP REQUIRED)
if (NOT TARGET Protobuf)
    find_package(Protobuf REQUIRED)
endif()
if(NOT TARGET PCL)
    find_package(PCL REQUIRED COMPONENTS common io)
endif()
find_package(Threads REQUIRED)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MACOSX_RPATH 0)
# recompile with -fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-std=c++14 -Wall -Wextra)

# Compile protobuf files
set(PROTO_DEFNS proto/voxblox/Block.proto
                proto/voxblox/Layer.proto)
set(BASE_PATH "proto")
set(PROTOBUF_COMPILE_PYTHON true)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

# Add voxblox library
set(${PROJECT_NAME}_SRCS
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} Eigen3::Eigen  ${PCL_LIBRARIES} ${PROTOBUF_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

# IDEs should put the headers in a nice place
source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})

install(TARGETS ${PROJECT_NAME}
	EXPORT voxbloxTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

INSTALL(EXPORT voxbloxTargets
	FILE "voxbloxTargets.cmake"
	NAMESPACE voxblox::
	DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/voxbloxConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/voxbloxConfig.cmake"
	INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/voxbloxConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

INSTALL(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/voxbloxConfig.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/voxbloxConfigVersion.cmake"
	DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)
