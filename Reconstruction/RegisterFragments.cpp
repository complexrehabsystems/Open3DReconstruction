#include "RegisterFragments.h"
#include "Helpers.h"
#include "Common.h"
#include "OptimizePoseGraph.h"
#include <Windows.h>

#include <Open3D/Core/Core.h>
#include <Open3D/Core/Registration/FastGlobalRegistration.h>
#include <Open3D/Core/Registration/Registration.h>
#include <Open3D/IO/IO.h>
#include <Open3D/Visualization/Visualization.h>

#include <iostream>
#include <tuple>
#include <cstdio>

using namespace open3d;


RegisterFragments::RegisterFragments ()
{
}


RegisterFragments::~RegisterFragments ()
{
}

std::tuple<std::shared_ptr<PointCloud>, std::shared_ptr<Feature>> PreprocessPointCloud ( PointCloud pcd )
{
  auto pcd_down = VoxelDownSample ( pcd, 0.05 );

  EstimateNormals ( *pcd_down, KDTreeSearchParamHybrid ( 0.1, 30 ) );

  auto pcd_fpfh = ComputeFPFHFeature ( *pcd_down, KDTreeSearchParamHybrid ( 0.25, 100 ) );

  return std::make_tuple ( pcd_down, pcd_fpfh );
}

std::tuple<bool, Eigen::Matrix4d> RegisterPointCloudFpfh (
  PointCloud& source,
  PointCloud& target,
  Feature& source_fpfh,
  Feature& target_fpfh )
{
  auto result = FastGlobalRegistration ( source, target, source_fpfh, target_fpfh, FastGlobalRegistrationOption ( 1.4, false, true, 0.07 ) );

  if (result.transformation_.trace () == 4.0)
  {
    return std::make_tuple ( false, Eigen::Matrix4d::Identity () );
  }
  else
  {
    return std::make_tuple ( true, result.transformation_ );
  }
}

std::tuple<bool, Eigen::Matrix4d> ComputeInitialRegistration (
  int s, int t,
  PointCloud& source_down,
  PointCloud& target_down,
  Feature& source_fpfh,
  Feature& target_fpfh,
  std::string path,
  bool draw_result = false)
{
  Eigen::Matrix4d transformation;
  bool success_reg;

  if (t == s + 1)
  {
    // odometry case
    PrintInfo ( "Using RGBD odometry\n" );
    PoseGraph pose_graph_frag;
    ReadPoseGraph ( Format ( path + template_fragment_posegraph_optimized, s ), pose_graph_frag );

    int n_nodes = pose_graph_frag.nodes_.size ();
    transformation = pose_graph_frag.nodes_[n_nodes - 1].pose_.inverse ();
  }
  else
  {
    PrintInfo ( "Register point cloud fpfh" );
    std::tie ( success_reg, transformation ) = RegisterPointCloudFpfh ( source_down, target_down, source_fpfh, target_fpfh );
    if (!success_reg)
    {
      return std::make_tuple ( false, Eigen::Matrix4d::Identity () );
    }
  }

  if (draw_result)
  {

  }

  return std::make_tuple ( true, transformation );
}



void RegisterPointCloud ( std::string path, std::vector<std::string> ply_file_names, bool draw_result = false )
{
  auto pose_graph = PoseGraph ();
  auto odometry = Eigen::Matrix4d::Identity ();
  auto info = Eigen::Matrix6d::Identity ();

  pose_graph.nodes_.push_back ( PoseGraphNode ( odometry ) );

  PointCloud source, target;
  std::shared_ptr<PointCloud> source_down, target_down;
  std::shared_ptr<Feature> source_fpfh, target_fpfh;
  bool success_global;
  Eigen::Matrix4d transformation_init;

  for (int s = 0; s < ply_file_names.size (); s++)
  {
    for (int t = s + 1; t < ply_file_names.size (); t++)
    {
      PrintInfo ( "Reading %s...\n", ply_file_names[s] );
      ReadPointCloud ( ply_file_names[s], source );
      PrintInfo ( "Reading %s...\n", ply_file_names[t] );
      ReadPointCloud ( ply_file_names[t], target );

      std::tie ( source_down, source_fpfh ) = PreprocessPointCloud ( source );
      std::tie ( target_down, target_fpfh ) = PreprocessPointCloud ( target );
      std::tie ( success_global, transformation_init ) = ComputeInitialRegistration ( 
        s, t, *source_down, *target_down, *source_fpfh, *target_fpfh, path, draw_result );

      if (!success_global)
        continue;

    }
  }
}

void RegisterFragments::Run ()
{
  std::string path = FullPath ( "..\\TestImages\\" );

  auto ply_file_names = GetFileList ( path + folder_fragment, ".ply" );
  MakeFolder ( path + folder_scene );

  RegisterPointCloud ( path, ply_file_names );

  auto optimize = OptimizePoseGraph ();
  optimize.OptimizePoseGraphForScene ( path );
}
