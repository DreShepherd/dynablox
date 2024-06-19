/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Qingwen Zhang (https://kin-zhang.github.io/)
 * @date: 2023-05-02 13:07
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map / Detection
 */

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "dynablox/dynablox.h"

int main(int argc, char** argv) {
  /* #region Initial  */
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(google::GLOG_INFO);

  if (argc < 3) {
      LOG(ERROR) << "Usage: " << argv[0] << " [pcd_folder] [config_file]";
      return 1;
  }
  std::string pcd_parent = argv[1];  // we assume that rawmap is in pcd_parent;
  std::string config_file = argv[2];
  int cnt = 1, run_max = 1;
  // check if the config_file exists
  if (!std::filesystem::exists(config_file)) {
    LOG(ERROR) << "Config file does not exist: " << config_file;
    return 1;
  }
  std::vector<std::string> filenames;
  for (const auto& entry : std::filesystem::directory_iterator(
           std::filesystem::path(pcd_parent) / "pcd")) {
    filenames.push_back(entry.path().string());
  }

  // sort the filenames
  std::sort(filenames.begin(), filenames.end());
  int total = filenames.size();
  if (argc > 3) {
    run_max = std::stoi(argv[3]);
    if (run_max == -1) {
      LOG(INFO) << "We will run all the frame in sequence, the total "
                   "number is: "
                << total;
      run_max = total + 1;
    }
  }
  /* #endregion */

  dynablox::MapUpdater map_updater(config_file);
  map_updater.timing.start("Total");
  for (const auto& filename : filenames) {
    map_updater.timing[0].start("One Scan Cost");
    if (cnt > 1 && !map_updater.getCfg().verbose_) {
      std::ostringstream log_msg;
      log_msg << "(" << cnt << "/" << run_max << ") Processing: " << filename
              << " Time Cost: " << map_updater.timing[0].lastSeconds() << "s";
      std::string spaces(10, ' ');
      log_msg << spaces;
      // std::cout <<log_msg.str()<<std::endl;
      std::cout << "\r" << log_msg.str() << std::flush;
    }

    if (filename.substr(filename.size() - 4) != ".pcd") continue;

    pcl::PointCloud<PointType>::Ptr pcd(new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile<PointType>(filename, *pcd);
    map_updater.run(pcd);
    map_updater.timing[0].stop();
    cnt++;
    if (cnt > run_max) break;
  }
  map_updater.saveMap(pcd_parent);
  map_updater.timing.stop();
  map_updater.timing.print("Dynablox " /*title*/, true /*color*/,
                           true /*bold*/);

  return 0;
}