#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/filesystem.hpp>

pcl::PointCloud<pcl::PointXYZI>::Ptr load_kitti_cloud(const std::string& filename) {
  FILE* file = fopen(filename.c_str(), "rb");
  if(!file) {
    std::cerr << "error: failed to load " << filename << std::endl;
    return nullptr;
  }

  std::vector<float> buffer(1000000);
  size_t num_points = fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
  fclose(file);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  cloud->resize(num_points);

  for(int i = 0; i < num_points; i++) {
    auto& pt = cloud->at(i);
    pt.x = buffer[i * 4];
    pt.y = buffer[i * 4 + 1];
    pt.z = buffer[i * 4 + 2];
    pt.intensity = buffer[i * 4 + 3];
  }

  return cloud;
}

std::vector<std::string> list_cloud_files(const std::string& sequence_path) {
  boost::filesystem::directory_iterator itr(sequence_path);
  boost::filesystem::directory_iterator end;

  std::vector<std::string> filenames;
  for(itr; itr != end; itr++) {
    if(itr->path().extension() == ".bin") {
      filenames.push_back(itr->path().string());
    }
  }

  std::sort(filenames.begin(), filenames.end());
  return filenames;
}
