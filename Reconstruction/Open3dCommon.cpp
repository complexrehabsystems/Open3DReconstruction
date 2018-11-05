#include "Open3dCommon.h"

using namespace open3d;

std::tuple<std::shared_ptr<PointCloud>, std::shared_ptr<Feature>> PreprocessPointCloud ( 
  PointCloud& pcd, 
  bool downsample )
{
  if (downsample)
  {

    auto pcd_down = VoxelDownSample ( pcd, 0.05 );

    EstimateNormals ( *pcd_down, KDTreeSearchParamHybrid ( 0.1, 30 ) );

    auto pcd_fpfh = ComputeFPFHFeature ( *pcd_down, KDTreeSearchParamHybrid ( 0.25, 100 ) );

    return std::make_tuple ( pcd_down, pcd_fpfh );
  }
  else
  {
    EstimateNormals ( pcd, KDTreeSearchParamHybrid ( 0.1, 30 ) );

    auto pcd_fpfh = ComputeFPFHFeature ( pcd, KDTreeSearchParamHybrid ( 0.25, 100 ) );

    return std::make_tuple ( std::make_shared<PointCloud>(pcd), pcd_fpfh );
  }
}

void VisualizeRegistration ( 
  const open3d::PointCloud &source,
  const open3d::PointCloud &target, 
  const Eigen::Matrix4d &Transformation,
  bool colored)
{
  std::shared_ptr<PointCloud> source_transformed_ptr ( new PointCloud );
  std::shared_ptr<PointCloud> target_ptr ( new PointCloud );
  *source_transformed_ptr = source;
  *target_ptr = target;

  if (colored)
  {
    source_transformed_ptr->PaintUniformColor ( Eigen::Vector3d ( 1, 0.706, 0 ) );
    target_ptr->PaintUniformColor ( Eigen::Vector3d ( 0, 0.651, 0.929 ) );
  }

  source_transformed_ptr->Transform ( Transformation );
  DrawGeometries ( { source_transformed_ptr, target_ptr }, "Registration result" );
}

void VisualizeRegistration (
  const open3d::TriangleMesh &source,
  const Eigen::Matrix4d &Transformation,
  bool colored )
{
  std::shared_ptr<TriangleMesh> source_transformed_ptr ( new TriangleMesh );
  *source_transformed_ptr = source;

  if (colored)
  {
    source_transformed_ptr->PaintUniformColor ( Eigen::Vector3d ( 1, 0.706, 0 ) );
  }

  source_transformed_ptr->Transform ( Transformation );
  DrawGeometries ( { source_transformed_ptr }, "Registration result" );
}

// colored pointcloud registration
// This is implementation of following paper
// J.Park, Q. - Y.Zhou, V.Koltun,
// Colored Point Cloud Registration Revisited, ICCV 2017
std::tuple< std::shared_ptr<RegistrationResult>, Eigen::Matrix6d> RegisterColoredPointCloudICP (
  const PointCloud& source,
  const PointCloud& target,
  Eigen::Matrix4d init_transformation/* = Eigen::Matrix4d::Identity ()*/,
  std::vector<double> voxel_radius /*= { 0.05, 0.025, 0.0125 }*/,
  std::vector<int> max_iter /*= { 50, 30, 14 }*/
)
{
  auto current_transformation = init_transformation;
  RegistrationResult result_icp;

  for (int scale = 0; scale < max_iter.size (); scale++)
  {
    auto iter = max_iter[scale];
    auto radius = voxel_radius[scale];
    auto source_down = VoxelDownSample ( source, radius );
    auto target_down = VoxelDownSample ( target, radius );

    EstimateNormals ( *source_down, KDTreeSearchParamHybrid ( radius * 2, 30 ) );
    EstimateNormals ( *target_down, KDTreeSearchParamHybrid ( radius * 2, 30 ) );

    result_icp = RegistrationICP (
      *source_down, *target_down, radius,
      current_transformation,
      TransformationEstimationPointToPoint ( false ),
      ICPConvergenceCriteria ( 1e-6, 1e-6, iter ) );
    current_transformation = result_icp.transformation_;
  }

  auto information_matrix = GetInformationMatrixFromPointClouds ( source, target, 0.07, result_icp.transformation_ );

  auto shared = std::make_shared< RegistrationResult > ( result_icp );

  return std::tie ( shared, information_matrix );
}
