#include "featureAssociation.h"
#include "imageProjection.h"
#include "mapOptimization.h"
#include "transformFusion.h"
#include "kitti.hpp"

#include <chrono>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lego_loam");
  ros::NodeHandle nh("~");

  const std::string sequence_path = nh.param<std::string>("sequence_path", "");
  const std::string dst_traj_path = nh.param<std::string>("dst_traj_path", "/tmp/traj.txt");

  std::vector<std::string> filenames = list_cloud_files(sequence_path + "/velodyne");
  std::vector<std::unordered_map<std::string, double>> best_params(filenames.size());

  for(int i=0; i<filenames.size(); i++) {
    if(!nh.param<bool>("adaptive", false)) {
      break;
    }

    std::ifstream ifs((boost::format("%s/lego_params/%06d.txt") % sequence_path % i).str());
    if(!ifs) {
      continue;
    }

    std::string line;

    while(!ifs.eof() && std::getline(ifs, line) && !line.empty()) {
      std::stringstream sst(line);

      std::string token;
      double value;
      sst >> token >> value;

      std::string label = token.substr(0, token.size() - 1);
      if(label.find("segment") != std::string::npos) {
        label = "/lego_loam/imageProjection/" + label;
      } else {
        label = "/lego_loam/featureAssociation/" + label;
      }

      best_params[i][label] = value;
    }
  }

  Channel<ProjectionOut> projection_out_channel(true);
  Channel<AssociationOut> association_out_channel(true);

  ImageProjection IP(nh, projection_out_channel);

  FeatureAssociation FA(nh, projection_out_channel, association_out_channel);
  MapOptimization MO(nh, association_out_channel);
  TransformFusion TF(nh);

  std::vector<bool> odom_valid(filenames.size(), false);
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> odometry(filenames.size(), Eigen::Isometry3d::Identity());
  odom_valid[0] = true;

  TF.odom_callback = [&](const nav_msgs::Odometry& odom) {
    int frame_id = static_cast<int>(odom.header.stamp.toSec() - 1000);
    odom_valid[frame_id] = true;

    const auto& pose = odom.pose.pose;
    odometry[frame_id].linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
    odometry[frame_id].translation() << pose.position.x, pose.position.y, pose.position.z;
  };

  ROS_INFO("\033[1;32m---->\033[0m LeGO-LOAM Started.");

  auto start_real_time = std::chrono::high_resolution_clock::now();
  auto start_sim_time = ros::Time(1000);

  auto prev_real_time = start_real_time;
  auto prev_sim_time = start_sim_time;

  auto clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock",1);

  for(int i=0; i<filenames.size(); i++) {
    ROS_INFO_STREAM(i << "/" << filenames.size());

    if(!best_params[i].empty()) {
      for(const auto& param : best_params[i]) {
        nh.setParam(param.first, param.second);
      }
    }

    auto cloud = load_kitti_cloud(filenames[i]);
    cloud->header.frame_id = "velodyne";

    std_msgs::Header header;
    header.frame_id = "velodyne";
    header.stamp = ros::Time(i + 1000.0);

    IP.cloudHandler(header, cloud);

    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = header.stamp;
    clock_publisher.publish( clock_msg );

    auto real_time = std::chrono::high_resolution_clock::now();
    if( real_time - prev_real_time > std::chrono::seconds(5) ) {
      auto sim_time = header.stamp;
      auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time-prev_real_time).count()*0.001;
      auto delta_sim = (sim_time - prev_sim_time).toSec();
      ROS_INFO("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);
      prev_sim_time = sim_time;
      prev_real_time = real_time;
    }
    ros::spinOnce();
  }

  ros::WallDuration(2.0).sleep();
  // must be called to cleanup threads
  ros::shutdown();

  Eigen::Isometry3d lidar2cam = Eigen::Isometry3d::Identity();
  lidar2cam.linear() = Eigen::Quaterniond(0.5, -0.5, -0.5, -0.5).toRotationMatrix();
  Eigen::Isometry3d cam2lidar = lidar2cam.inverse();

  std::ofstream traj_ofs(dst_traj_path);
  for(int i=0; i<filenames.size(); i++) {
    if(!odom_valid[i]) {
      odometry[i] = odometry[i - 1];
      std::cout << "frame " << i << " is invalid" << std::endl;
    }

    Eigen::Isometry3d pose = cam2lidar * odometry[i] * lidar2cam;

    for(int row = 0; row < 3; row++) {
      for(int col = 0; col < 4; col++) {
        if(row || col) {
          traj_ofs << " ";
        }

        traj_ofs << boost::format("%.6f") % pose(row, col);
      }
    }

    traj_ofs << std::endl;
  }
  traj_ofs.close();

  return 0;
}