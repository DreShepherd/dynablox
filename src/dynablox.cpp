/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Qingwen Zhang (https://kin-zhang.github.io/)
 * @date: 2023-05-02 17:03
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map / Detection
 */
#include "dynablox/dynablox.h"

#include <future>
#include <mutex>

#include <pcl/common/transforms.h>

#include "dynablox/index_getter.h"

namespace dynablox {
MapUpdater::MapUpdater(const std::string& config_file_path) {
  yconfig = YAML::LoadFile(config_file_path);
  MapUpdater::setConfig();

  // reset and initial
  Dynamic_Cloud_.reset(new pcl::PointCloud<PointType>);
  Static_Cloud_.reset(new pcl::PointCloud<PointType>);
  tsdf_mapper_ = std::make_shared<voxblox::TsdfMapper>(config_.voxblox_integrator_);

  // (Note from Kin) I don't know why the origin one can directly use raw ptr without any
  // problem Since it will cause double free problem. I used shared_ptr to avoid this
  tsdf_layer_ = std::shared_ptr<TsdfLayer>(tsdf_mapper_->getTsdfMapPtr()->getTsdfLayerPtr(), [](TsdfLayer*) {});

  // function
  clustering_ = std::make_shared<Clustering>(config_.clustering, tsdf_layer_);
  ever_free_integrator_ = std::make_shared<EverFreeIntegrator>(config_.ever_free_integrator_, tsdf_layer_);
}

void MapUpdater::setConfig() {
  // clang-format off
  // Set Motion Detector Parameters
  config_.num_threads = yconfig["num_threads"].as<int>();
  config_.verbose_ = yconfig["verbose"].as<bool>();

  // Set Preprocessing Parameters
  config_.min_range_m = yconfig["preprocessing"]["min_range"].as<double>();
  
  if(yconfig["preprocessing"]["max_range"].as<double>()>0)
    config_.max_range_m = yconfig["preprocessing"]["max_range"].as<double>();
  else
    config_.max_range_m = std::numeric_limits<double>::infinity();

  // Set Ever-Free Integration Parameters
  config_.ever_free_integrator_.counter_to_reset = yconfig["ever_free_integrator"]["counter_to_reset"].as<int>();
  config_.ever_free_integrator_.temporal_buffer = yconfig["ever_free_integrator"]["temporal_buffer"].as<int>();
  config_.ever_free_integrator_.burn_in_period = yconfig["ever_free_integrator"]["burn_in_period"].as<int>();
  config_.ever_free_integrator_.tsdf_occupancy_threshold = yconfig["ever_free_integrator"]["tsdf_occupancy_threshold"].as<float>();
  config_.ever_free_integrator_.neighbor_connectivity = yconfig["ever_free_integrator"]["neighbor_connectivity"].as<int>();
  config_.ever_free_integrator_.num_threads = &config_.num_threads;

  // Set Clustering Parameters
  config_.clustering.min_cluster_size = yconfig["clustering"]["min_cluster_size"].as<int>();
  config_.clustering.max_cluster_size = yconfig["clustering"]["max_cluster_size"].as<int>();
  config_.clustering.min_extent = yconfig["clustering"]["min_extent"].as<float>();
  config_.clustering.max_extent = yconfig["clustering"]["max_extent"].as<float>();
  config_.clustering.neighbor_connectivity = yconfig["clustering"]["neighbor_connectivity"].as<int>();
  config_.clustering.grow_clusters_twice = yconfig["clustering"]["grow_clusters_twice"].as<bool>();
  config_.clustering.check_cluster_separation_exact = yconfig["clustering"]["check_cluster_separation_exact"].as<bool>();
  config_.clustering.min_cluster_separation = yconfig["clustering"]["min_cluster_separation"].as<float>();

  // Set Tracking Parameters
  config_.min_track_duration = yconfig["tracking"]["min_track_duration"].as<int>();
  config_.max_tracking_distance = yconfig["tracking"]["max_tracking_distance"].as<float>();

  // Set Voxblox Parameters
  config_.voxblox_integrator_.tsdf_voxel_size_ = yconfig["voxblox"]["tsdf_voxel_size"].as<float>();
  config_.voxblox_integrator_.truncation_distance_ = yconfig["voxblox"]["truncation_distance"].as<float>();
  config_.voxblox_integrator_.tsdf_voxels_per_side_ = yconfig["voxblox"]["tsdf_voxels_per_side"].as<int>();
  config_.voxblox_integrator_.max_weight_ = yconfig["voxblox"]["max_weight"].as<int>();
  config_.voxblox_integrator_.tsdf_methods = yconfig["voxblox"]["method"].as<std::string>();
  config_.voxblox_integrator_.sensor_horizontal_resolution_ = yconfig["voxblox"]["sensor_horizontal_resolution"].as<int>();
  config_.voxblox_integrator_.sensor_vertical_resolution_ = yconfig["voxblox"]["sensor_vertical_resolution"].as<int>();
  config_.voxblox_integrator_.sensor_vertical_field_of_view_degrees_ = yconfig["voxblox"]["sensor_vertical_field_of_view_degrees"].as<float>();
  config_.voxblox_integrator_.use_const_weight_ = yconfig["voxblox"]["use_const_weight"].as<bool>();
  config_.voxblox_integrator_.max_range_m = &config_.max_range_m;
  config_.voxblox_integrator_.min_range_m = &config_.min_range_m;
  config_.voxblox_integrator_.verbose_ = &config_.verbose_;
  // clang-format on
}

void MapUpdater::run(pcl::PointCloud<PointType>::Ptr const& single_pc) {
  Cloud cloud = *single_pc;
  CloudInfo cloud_info;

  // read pose in VIEWPOINT Field in pcd
  cloud_info.sensor_position.x = single_pc->sensor_origin_[0];
  cloud_info.sensor_position.y = single_pc->sensor_origin_[1];
  cloud_info.sensor_position.z = single_pc->sensor_origin_[2];
  // set T_World_Sensor based on sensor pose
  Eigen::Matrix4f T_Sensor_World = Eigen::Matrix4f::Identity();
  T_Sensor_World.block<3, 3>(0, 0) = single_pc->sensor_orientation_.toRotationMatrix();
  T_Sensor_World.block<3, 1>(0, 3) = single_pc->sensor_origin_.head<3>();
  voxblox::Transformation T_S_W(T_Sensor_World.cast<float>());

  frame_counter_++;
  timing[1].start("Process Pointcloud");
  processPointCloud(cloud, cloud_info);
  timing[1].stop();

  timing[2].start("Index Setup");
  BlockToPointMap point_map;
  std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices;
  setUpPointMap(cloud, point_map, occupied_ever_free_voxel_indices, cloud_info);
  timing[2].stop();

  timing[3].start("Clustering");
  Clusters clusters = clustering_->performClustering(point_map, occupied_ever_free_voxel_indices, frame_counter_, cloud,
                                                     cloud_info, timing);
  timing[3].stop();

  timing[4].start("Tracking");
  Tracking(cloud, clusters, cloud_info);
  timing[4].stop();

  timing[5].start("Update EverFree");
  ever_free_integrator_->updateEverFreeVoxels(frame_counter_, timing);
  timing[5].stop();

  timing[6].start("Integrate TSDF");
  Cloud origin_cloud;
  pcl::transformPointCloud(cloud, origin_cloud, T_Sensor_World.inverse());
  tsdf_mapper_->processPointCloudAndInsert(origin_cloud, T_S_W, timing);
  timing[6].stop();

  /* Note from Kin */
  /* optional 1: use the cluster info to decide
     recommend for real-time detection, lower score in clean task but high in IoU */ 
  // for (const Cluster& cluster : clusters) {
  //   for (int index : cluster.points){
  //     Dynamic_Cloud_->points.emplace_back(cloud[index].x, cloud[index].y, cloud[index].z);
  //   }
  // }
  // size_t i = 0;
  // for (const auto& point : cloud.points) {
  //   if (!cloud_info.points[i++].cluster_level_dynamic)
  //     Static_Cloud_->points.emplace_back(point.x, point.y, point.z);
  // }

  /* optional 2: use the ever_free info
     recommend for clean task, higher clean score than previous one */ 
  size_t i = 0;
  for (const auto& pt : cloud.points) {
    if (cloud_info.points[i++].ever_free_level_dynamic)
      Dynamic_Cloud_->points.emplace_back(pt.x, pt.y, pt.z);
    else
      Static_Cloud_->points.emplace_back(pt.x, pt.y, pt.z);
  }
}

void MapUpdater::saveMap(std::string const& folder_path) {
  pcl::PointCloud<PointType>::Ptr save_cloud;
  std::cout << std::endl;
  
  LOG(INFO) << "Saving Static map to " << folder_path
            << "\nPointcloud size: " << Static_Cloud_->points.size() << " points.";
  save_cloud = Static_Cloud_;

  if (save_cloud->points.size() > 0){
    LOG(INFO) << "Saving folder: " << folder_path + "/dynablox_output_static.pcd";
    pcl::io::savePCDFileBinary(folder_path + "/dynablox_output_static.pcd", *save_cloud);
  }
  
  LOG(INFO) << "Saving Dynamic map to " << folder_path
                << "\nPointcloud size: " << Dynamic_Cloud_->points.size() << " points.";
  save_cloud = Dynamic_Cloud_;

  if (save_cloud->points.size() > 0) {
        LOG(INFO) << "Saving folder: " << folder_path + "/dynablox_output_dynamic.pcd";
        pcl::io::savePCDFileBinary(folder_path + "/dynablox_output_dynamic.pcd", *save_cloud);
  }
}

void MapUpdater::Tracking(const Cloud& cloud, Clusters& clusters, CloudInfo& cloud_info) {
  timing[4][1].start("Cluster IDs");
  // Associate current to previous cluster ids.
  trackClusterIDs(cloud, clusters);
  timing[4][1].stop();
  // Label the cloud info.
  timing[4][2].start("Label Point");
  for (Cluster& cluster : clusters) {
    if (cluster.track_length >= config_.min_track_duration) {
      cluster.valid = true;
      for (int idx : cluster.points) {
        cloud_info.points[idx].object_level_dynamic = true;
      }
    }
  }
  timing[4][2].stop();
}

void MapUpdater::trackClusterIDs(const Cloud& cloud, Clusters& clusters) {
  // Compute the centroids of all clusters.
  std::vector<voxblox::Point> centroids(clusters.size());
  size_t i = 0;
  for (const Cluster& cluster : clusters) {
    voxblox::Point centroid = {0, 0, 0};
    for (int index : cluster.points) {
      const Point& point = cloud[index];
      centroid = centroid + voxblox::Point(point.x, point.y, point.z);
    }
    centroids[i] = centroid / cluster.points.size();
    ++i;
  }

  // Compute the distances of all clusters. [previous][current]->dist
  struct Association {
    float distance;
    int previous_id;
    int current_id;
  };

  std::vector<std::vector<Association>> distances(previous_centroids_.size());
  for (size_t i = 0; i < previous_centroids_.size(); ++i) {
    std::vector<Association>& d = distances[i];
    d.reserve(centroids.size());
    for (size_t j = 0; j < centroids.size(); ++j) {
      Association association;
      association.distance = (previous_centroids_[i] - centroids[j]).norm();
      association.previous_id = i;
      association.current_id = j;
      d.push_back(association);
    }
  }

  // Associate all previous ids until no more minimum distances exist.
  std::unordered_set<int> reused_ids;
  while (true) {
    // Find the minimum distance and IDs (exhaustively).
    float min = std::numeric_limits<float>::max();
    int prev_id = 0;
    int curr_id = 0;
    int erase_i = 0;
    int erase_j = 0;
    for (size_t i = 0u; i < distances.size(); ++i) {
      for (size_t j = 0u; j < distances[i].size(); ++j) {
        const Association& association = distances[i][j];
        if (association.distance < min) {
          min = association.distance;
          curr_id = association.current_id;
          prev_id = association.previous_id;
          erase_i = i;
          erase_j = j;
        }
      }
    }

    if (min > config_.max_tracking_distance) {
      // no more good fits.
      break;
    }

    // Update traked cluster and remove that match to search for next best.
    clusters[curr_id].id = previous_ids_[prev_id];
    clusters[curr_id].track_length = previous_track_lengths_[prev_id] + 1;
    reused_ids.insert(previous_ids_[prev_id]);
    distances.erase(distances.begin() + erase_i);
    for (auto& vec : distances) {
      vec.erase(vec.begin() + erase_j);
    }
  }

  // Fill in all remaining ids and track data.
  previous_centroids_ = centroids;
  previous_ids_.clear();
  previous_ids_.reserve(clusters.size());
  previous_track_lengths_.clear();
  previous_ids_.reserve(clusters.size());

  int id_counter = 0;
  for (Cluster& cluster : clusters) {
    if (cluster.id == -1) {
      // We need to replace it.
      while (reused_ids.find(id_counter) != reused_ids.end()) {
        id_counter++;
      }
      cluster.id = id_counter;
      id_counter++;
    }
    previous_ids_.push_back(cluster.id);
    previous_track_lengths_.push_back(cluster.track_length);
  }
}

bool MapUpdater::processPointCloud(pcl::PointCloud<PointType>& cloud, CloudInfo& cloud_info) {
  cloud_info.points = std::vector<PointInfo>(cloud.size());
  size_t i = 0;
  for (const auto& pt : cloud) {
    double delta_x = pt.x - cloud_info.sensor_position.x;
    double delta_y = pt.y - cloud_info.sensor_position.y;
    double delta_z = pt.z - cloud_info.sensor_position.z;
    const float norm = std::sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
    PointInfo& info = cloud_info.points.at(i);
    info.distance_to_sensor = norm;
    i++;
  }
  return true;
}

void MapUpdater::setUpPointMap(const pcl::PointCloud<PointType>& cloud, BlockToPointMap& point_map,
                               std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
                               CloudInfo& cloud_info) const {
  // Identifies for any LiDAR point the block it falls in and constructs the
  // hash-map block2points_map mapping each block to the LiDAR points that
  // fall into the block.
  const voxblox::HierarchicalIndexIntMap block2points_map = buildBlockToPointsMap(cloud);

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
        this->blockwiseBuildPointMap(cloud, block_index, block2points_map.at(block_index), result,
                                     local_occupied_indices, cloud_info);
        local_point_map.insert(std::pair(block_index, result));
      }

      // After processing is done add data to the output map.
      std::lock_guard<std::mutex> lock(aggregate_results_mutex);
      occupied_ever_free_voxel_indices.insert(occupied_ever_free_voxel_indices.end(), local_occupied_indices.begin(),
                                              local_occupied_indices.end());
      point_map.merge(local_point_map);
    }));
  }
  for (auto& thread : threads) {
    thread.get();
  }
}

voxblox::HierarchicalIndexIntMap MapUpdater::buildBlockToPointsMap(const pcl::PointCloud<PointType>& cloud) const {
  voxblox::HierarchicalIndexIntMap result;

  int i = 0;
  for (const auto& point : cloud) {
    voxblox::Point coord(point.x, point.y, point.z);
    const BlockIndex blockindex = tsdf_layer_->computeBlockIndexFromCoordinates(coord);
    result[blockindex].push_back(i);
    i++;
  }
  return result;
}

void MapUpdater::blockwiseBuildPointMap(const pcl::PointCloud<PointType>& cloud, const BlockIndex& block_index,
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
    const VoxelIndex voxel_index = tsdf_block->computeVoxelIndexFromCoordinates(coords);
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
    TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByVoxelIndex(voxel_points_pair.first);
    tsdf_voxel.last_lidar_occupied = frame_counter_;

    // This voxel attribute is used in the voxel clustering method: it
    // signalizes that a currently occupied voxel has not yet been clustered
    tsdf_voxel.clustering_processed = false;

    // The set of occupied_ever_free_voxel_indices allows for fast access of
    // the seed voxels in the voxel clustering
    if (tsdf_voxel.ever_free) {
      occupied_ever_free_voxel_indices.push_back(std::make_pair(block_index, voxel_points_pair.first));
    }
  }
}
}  // namespace dynablox