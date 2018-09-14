#pragma once

#include <Open3D/Core/Core.h>
#include <Open3D/Core/Registration/FastGlobalRegistration.h>
#include <Open3D/IO/IO.h>
#include <Open3D/Visualization/Visualization.h>

#include <tuple>

std::tuple<std::shared_ptr<open3d::PointCloud>, std::shared_ptr<open3d::Feature>> PreprocessPointCloud ( open3d::PointCloud& pcd );

void VisualizeRegistration (
  const open3d::PointCloud &source,
  const open3d::PointCloud &target,
  const Eigen::Matrix4d &Transformation,
  bool colored = false);

void VisualizeRegistration (
  const open3d::TriangleMesh &source,
  const Eigen::Matrix4d &Transformation,
  bool colored = false );

std::tuple< std::shared_ptr<open3d::RegistrationResult>, Eigen::Matrix6d> RegisterColoredPointCloudICP (
  const open3d::PointCloud& source,
  const open3d::PointCloud& target,
  Eigen::Matrix4d init_transformation = Eigen::Matrix4d::Identity (),
  std::vector<double> voxel_radius = { 0.05, 0.025, 0.0125 },
  std::vector<int> max_iter = { 50, 30, 14 }
);


