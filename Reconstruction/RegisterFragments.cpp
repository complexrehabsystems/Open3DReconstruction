#include "RegisterFragments.h"
#include "Helpers.h"
#include "Common.h"
#include "OptimizePoseGraph.h"
#include <Windows.h>
#include "Open3dCommon.h"

#include <Open3D/Core/Core.h>
#include <Open3D/Core/Registration/FastGlobalRegistration.h>
#include <Open3D/IO/IO.h>

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
  bool draw_result = false )
{
  Eigen::Matrix4d transformation;
  bool success_reg;

  if (t == s + 1)
  {
    // odometry case
    PrintInfo ( "Using RGBD odometry\n" );
    PoseGraph pose_graph_frag;
    ReadPoseGraph ( Format ( path + template_fragment_posegraph_optimized, s ), pose_graph_frag );

    int n_nodes = (int)pose_graph_frag.nodes_.size ();
    transformation = pose_graph_frag.nodes_[n_nodes - 1].pose_.inverse ();
  }
  else
  {
    // loop closure
    PrintInfo ( "Register point cloud fpfh\n" );
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



std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> LocalRefinement (
  int s, int t,
  PointCloud& source,
  PointCloud& target,
  Eigen::Matrix4d& transformation_init,
  bool draw_result = false )
{
  Eigen::Matrix4d transformation;
  Eigen::Matrix6d information;

  std::shared_ptr<RegistrationResult> result;

  if (t == s + 1)
  {
    // odometry case
    std::tie ( result, information ) = RegisterColoredPointCloudICP ( source, target, transformation_init, { 0.0125 }, { 30 } );
  }
  else
  {
    // loop closure case
    std::tie ( result, information ) = RegisterColoredPointCloudICP ( source, target, transformation_init);
  }

  transformation = result->transformation_;

  bool success_local = false;

  if (information(5,5) / (double)std::min ( (double)source.points_.size (), (double)target.points_.size () ) > 0.3)
  {
    success_local = true;
  }

  if(draw_result)
  { }
  
  return std::make_tuple (success_local, transformation, information );
}

std::tuple<Eigen::Matrix4d, PoseGraph> UpdateOdometryPoseGraph (
  int s, int t,
  Eigen::Matrix4d& transformation,
  Eigen::Matrix6d& information,
  Eigen::Matrix4d& odometry,
  PoseGraph pose_graph
)
{
  if (t == s + 1)
  {
    // odometry case
    odometry = transformation * odometry;
    auto odometry_inv = odometry.inverse ();
    pose_graph.nodes_.push_back ( PoseGraphNode ( odometry_inv ) );
    pose_graph.edges_.push_back ( PoseGraphEdge ( s, t, transformation, information, false ) );
  }
  else
  {
    // loop closure case
    pose_graph.edges_.push_back ( PoseGraphEdge ( s, t, transformation, information, true ) );
  }

  return std::make_tuple ( odometry, pose_graph );
}

void RegisterPointCloud ( std::string path, std::vector<std::string> ply_file_names, bool draw_result = false )
{
  PoseGraph pose_graph;
  Eigen::Matrix4d odometry = Eigen::Matrix4d::Identity ();

  pose_graph.nodes_.push_back ( PoseGraphNode ( odometry ) );

  PointCloud source, target;
  std::shared_ptr<PointCloud> source_down, target_down;
  std::shared_ptr<Feature> source_fpfh, target_fpfh;
  bool success_global, success_local;
  Eigen::Matrix4d transformation_init, transformation_icp;
  Eigen::Matrix6d information_icp;

  for (int s = 0; s < ply_file_names.size (); s++)
  {
    for (int t = s + 1; t < ply_file_names.size (); t++)
    {
      PrintInfo ( "Reading %s...\n", ply_file_names[s].c_str() );
      ReadPointCloud ( ply_file_names[s], source );
      PrintInfo ( "Reading %s...\n", ply_file_names[t].c_str () );
      ReadPointCloud ( ply_file_names[t], target );

      std::tie ( source_down, source_fpfh ) = PreprocessPointCloud ( source );
      std::tie ( target_down, target_fpfh ) = PreprocessPointCloud ( target );
      std::tie ( success_global, transformation_init ) = ComputeInitialRegistration ( 
        s, t, *source_down, *target_down, *source_fpfh, *target_fpfh, path, draw_result );

      if (!success_global)
        continue;

      std::tie ( success_local, transformation_icp, information_icp ) = LocalRefinement (
        s, t, source, target, transformation_init, draw_result );

      if (!success_local)
        continue;

      std::tie ( odometry, pose_graph ) = UpdateOdometryPoseGraph (
        s, t, transformation_icp, information_icp,
        odometry, pose_graph );

    }
  }

  WritePoseGraph ( path + template_global_posegraph, pose_graph );
}

void RegisterFragments::Run (std::string path)
{
  ScopeTimer timer ( "Register Fragments" );
  auto ply_file_names = GetFileList ( path + folder_fragment, ".ply" );
  MakeFolder ( path + folder_scene );

  RegisterPointCloud ( path, ply_file_names );

  auto optimize = OptimizePoseGraph ();
  optimize.OptimizePoseGraphForScene ( path );
}
