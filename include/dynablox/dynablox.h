/**
 * Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-05-02 13:19
 * @details: No ROS version, speed up the process, check our benchmark in dufomap
 */
#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <voxblox/core/block.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include "common/utils.h"
#include "common/timing.hpp"
#include "dynablox/types.h"

namespace dynablox {

class MapUpdater {
public:
    MapUpdater(const std::string &config_file_path);
    virtual ~MapUpdater() = default;
    
    ufo::Timing timing;
    
    void setConfig();
    void run(pcl::PointCloud<PointType>::Ptr const& single_pc);

private:
    common::Config config_;
    YAML::Node yconfig;
    std::shared_ptr<TsdfLayer> tsdf_layer_;
    size_t frame_counter_ = 0;

    bool processPointcloud(pcl::PointCloud<PointType>::Ptr const& single_pc,
                           pcl::PointCloud<PointType>& cloud, CloudInfo& cloud_info);
    void setUpPointMap(const pcl::PointCloud<PointType>& cloud, BlockToPointMap& point_map,
                    std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
                    CloudInfo& cloud_info) const;

    voxblox::HierarchicalIndexIntMap buildBlockToPointsMap(
            const pcl::PointCloud<PointType>& cloud) const;
    void blockwiseBuildPointMap(
        const pcl::PointCloud<PointType>& cloud, const BlockIndex& block_index,
        const voxblox::AlignedVector<size_t>& points_in_block,
        VoxelToPointMap& voxel_map,
        std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
        CloudInfo& cloud_info) const;
};
}  // namespace octomap