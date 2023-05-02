/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-05-02 17:03
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map / Detection
 */
#include <future>
#include <mutex>

#include "dynablox/dynablox.h"
#include "dynablox/index_getter.h"


namespace dynablox {
    MapUpdater::MapUpdater(const std::string &config_file_path ){
        yconfig = YAML::LoadFile(config_file_path);
        MapUpdater::setConfig();
    }
    
    void MapUpdater::setConfig(){
        config_.num_threads = yconfig["num_threads"].as<int>();
    }

    void MapUpdater::run(pcl::PointCloud<PointType>::Ptr const& single_pc){
        pcl::PointCloud<PointType> cloud;
        CloudInfo cloud_info;

        frame_counter_++;
        timing[1].start("Process Pointcloud");
        processPointcloud(single_pc, cloud, cloud_info);
        timing[1].stop();

        timing[2].start("Index Setup");
        BlockToPointMap point_map;
        std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices;
        setUpPointMap(cloud, point_map, occupied_ever_free_voxel_indices, cloud_info);
        timing[2].stop();

        timing[3].start("Clustering");
        // TODO but the cluster inside have tsdfserver with ros tightly.
        // Need sometime to figure out.
        timing[3].stop();
    }

    void MapUpdater::setUpPointMap(
        const pcl::PointCloud<PointType>& cloud, BlockToPointMap& point_map,
        std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
        CloudInfo& cloud_info) const {
        // Identifies for any LiDAR point the block it falls in and constructs the
        // hash-map block2points_map mapping each block to the LiDAR points that
        // fall into the block.
        const voxblox::HierarchicalIndexIntMap block2points_map =
            buildBlockToPointsMap(cloud);

        // Builds the voxel2point-map in parallel blockwise.
        std::vector<BlockIndex> block_indices(block2points_map.size());
        size_t i = 0;
        for (const auto& block : block2points_map) {
            block_indices[i] = block.first;
            ++i;
        }
        IndexGetter<BlockIndex> index_getter(block_indices);
        std::vector<std::future<void>> threads;
        std::mutex aggregate_results_mutex;
        for (int i = 0; i < config_.num_threads; ++i) {
            threads.emplace_back(std::async(std::launch::async, [&]() {
            // Data to store results.
            BlockIndex block_index;
            std::vector<voxblox::VoxelKey> local_occupied_indices;
            BlockToPointMap local_point_map;

            // Process until no more blocks.
            while (index_getter.getNextIndex(&block_index)) {
                VoxelToPointMap result;
                this->blockwiseBuildPointMap(cloud, block_index,
                                            block2points_map.at(block_index), result,
                                            local_occupied_indices, cloud_info);
                local_point_map.insert(std::pair(block_index, result));
            }

            // After processing is done add data to the output map.
            std::lock_guard<std::mutex> lock(aggregate_results_mutex);
            occupied_ever_free_voxel_indices.insert(
                occupied_ever_free_voxel_indices.end(),
                local_occupied_indices.begin(), local_occupied_indices.end());
            point_map.merge(local_point_map);
            }));
        }
        for (auto& thread : threads) {
            thread.get();
        }
    }

    bool MapUpdater::processPointcloud(pcl::PointCloud<PointType>::Ptr const& single_pc,
                           pcl::PointCloud<PointType>& cloud, CloudInfo& cloud_info){
        // read pose in VIEWPOINT Field in pcd
        cloud_info.sensor_position.x = single_pc->sensor_origin_[0];
        cloud_info.sensor_position.y = single_pc->sensor_origin_[1];
        cloud_info.sensor_position.z = single_pc->sensor_origin_[2];
        
        cloud_info.points = std::vector<PointInfo>(cloud.size());
        size_t i = 0;
        for (const auto& point : cloud) {
            float px = point.x - cloud_info.sensor_position.x;
            float py = point.y - cloud_info.sensor_position.y;
            float pz = point.z - cloud_info.sensor_position.z;

            const float norm =
                std::sqrt(px * px + py * py + pz * pz);
            PointInfo& info = cloud_info.points.at(i);
            info.distance_to_sensor = norm;
            i++;
        }
        return true;
    }
    
    voxblox::HierarchicalIndexIntMap MapUpdater::buildBlockToPointsMap(
        const pcl::PointCloud<PointType>& cloud) const {
        voxblox::HierarchicalIndexIntMap result;

        int i = 0;
        for (const auto& point : cloud) {
            voxblox::Point coord(point.x, point.y, point.z);
            const BlockIndex blockindex =
                tsdf_layer_->computeBlockIndexFromCoordinates(coord);
            result[blockindex].push_back(i);
            i++;
        }
        return result;
    }

    void MapUpdater::blockwiseBuildPointMap(
        const pcl::PointCloud<PointType>& cloud, const BlockIndex& block_index,
        const voxblox::AlignedVector<size_t>& points_in_block,
        VoxelToPointMap& voxel_map,
        std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
        CloudInfo& cloud_info) const {
    // Get the block.
    TsdfBlock::Ptr tsdf_block = tsdf_layer_->getBlockPtrByIndex(block_index);
    if (!tsdf_block) {
        return;
    }

    // Create a mapping of each voxel index to the points it contains.
    for (size_t i : points_in_block) {
        const PointType& point = cloud[i];
        const voxblox::Point coords(point.x, point.y, point.z);
        const VoxelIndex voxel_index =
            tsdf_block->computeVoxelIndexFromCoordinates(coords);
        if (!tsdf_block->isValidVoxelIndex(voxel_index)) {
        continue;
        }
        voxel_map[voxel_index].push_back(i);

        // EverFree detection flag at the same time, since we anyways lookup
        // voxels.
        if (tsdf_block->getVoxelByVoxelIndex(voxel_index).ever_free) {
        cloud_info.points.at(i).ever_free_level_dynamic = true;
        }
    }

    // Update the voxel status of the currently occupied voxels.
    for (const auto& voxel_points_pair : voxel_map) {
        TsdfVoxel& tsdf_voxel =
            tsdf_block->getVoxelByVoxelIndex(voxel_points_pair.first);
        tsdf_voxel.last_lidar_occupied = frame_counter_;

        // This voxel attribute is used in the voxel clustering method: it
        // signalizes that a currently occupied voxel has not yet been clustered
        tsdf_voxel.clustering_processed = false;

        // The set of occupied_ever_free_voxel_indices allows for fast access of
        // the seed voxels in the voxel clustering
        if (tsdf_voxel.ever_free) {
        occupied_ever_free_voxel_indices.push_back(
            std::make_pair(block_index, voxel_points_pair.first));
        }
    }
    }
}