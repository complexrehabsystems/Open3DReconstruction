#include "MakeFragments.h"
#include "Helpers.h"
#include "Common.h"
#include "OptimizePoseGraph.h"

#include <Open3D/Core/Core.h>
#include <Open3D/Core/Registration/FastGlobalRegistration.h>
#include <Open3D/IO/IO.h>
#include <Open3D/Visualization/Visualization.h>

#include <iostream>
#include <tuple>
#include <cstdio>

using namespace open3d;

int n_frames_per_fragment = 100;
int n_keyframes_per_n_frame = 5;

std::string folder_fragment = "fragments\\";
std::string template_fragment_posegraph = folder_fragment + "fragment_%03d.json";
std::string template_fragment_posegraph_optimized = folder_fragment + "fragment_optimized_%03d.json";

std::string template_fragment_mesh = folder_fragment + "fragment_%03d.ply";
std::string folder_scene = "scene\\";
std::string template_global_posegraph = folder_scene + "global_registration.json";
std::string template_global_posegraph_optimized = folder_scene + "global_registration_optimized.json";
std::string template_global_mesh = folder_scene + "integrated.ply";


MakeFragments::MakeFragments ()
{
}


MakeFragments::~MakeFragments ()
{
}

std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> RegisterOneRGBDPair (
  int s, int t,
  std::vector<std::string> colorFiles,
  std::vector<std::string> depthFiles,
  open3d::PinholeCameraIntrinsic & intrinsic )
{
  if (std::abs ( s - t ) != 1)
  {
    return std::make_tuple ( false, Eigen::Matrix4d::Identity (), Eigen::Matrix6d::Identity () );
  }

  auto color_s = CreateImageFromFile ( colorFiles[s] );
  auto depth_s = CreateImageFromFile ( depthFiles[s] );
  auto color_t = CreateImageFromFile ( colorFiles[t] );
  auto depth_t = CreateImageFromFile ( depthFiles[t] );

  auto rgbd_s = CreateRGBDImageFromColorAndDepth ( *color_s, *depth_s );
  auto rgbd_t = CreateRGBDImageFromColorAndDepth ( *color_t, *depth_t );

  Eigen::Matrix4d odo_init = Eigen::Matrix4d::Identity ();

  return ComputeRGBDOdometry ( *rgbd_s, *rgbd_t, intrinsic, odo_init, RGBDOdometryJacobianFromHybridTerm (), OdometryOption () );

}

std::tuple<std::vector<std::string>, std::vector<std::string>> ReadRGBDColorFiles ( std::string path )
{
  std::vector<std::string> colorFiles, depthFiles;

  colorFiles = GetFileList ( path + "rgb" );
  depthFiles = GetFileList ( path + "depth" );

  return std::make_tuple ( colorFiles, depthFiles );
}

std::shared_ptr<TriangleMesh> IntegrateRGBFramesForFragment (
  std::vector<std::string> colorFiles,
  std::vector<std::string> depthFiles,
  int fragment_id, int fragmentCount,
  std::string poseGraphName,
  PinholeCameraIntrinsic & intrinsic
)
{
  PoseGraph pose_graph;
  ReadPoseGraph ( poseGraphName, pose_graph );

  ScalableTSDFVolume volume ( 3.0 / 512.0, 0.04, TSDFVolumeColorType::RGB8 );

  int i_abs;

  Image color, depth;
  for (int i = 0; i < pose_graph.nodes_.size (); i++)
  {
    i_abs = fragment_id * n_frames_per_fragment + i;

    PrintInfo ( "Fragment %03d / %03d :: integrate rgbd frame %d (%d of %d).\n", fragment_id, fragmentCount - 1, i_abs, i + 1, pose_graph.nodes_.size());

    ReadImage ( colorFiles[i_abs], color );
    ReadImage ( depthFiles[i_abs], depth );

    auto rgbd = CreateRGBDImageFromColorAndDepth ( color, depth, 1000.0, 3.0, false );

    auto pose = pose_graph.nodes_[i].pose_;
    volume.Integrate ( *rgbd, intrinsic, pose.inverse () );
  }

  auto mesh = volume.ExtractTriangleMesh ();
  mesh->ComputeVertexNormals ();

  return mesh;
}

void MakeMeshForFragment (
  std::string path,
  std::vector<std::string> colorFiles,
  std::vector<std::string> depthFiles,
  int fragment_id, int fragmentCount,
  PinholeCameraIntrinsic & intrinsic )
{
  auto mesh = IntegrateRGBFramesForFragment (
    colorFiles, depthFiles, fragment_id, fragmentCount,
    Format ( path + template_fragment_posegraph_optimized, fragment_id ),
    intrinsic );

  auto mesh_name = Format ( path + template_fragment_mesh, fragment_id );

  WriteTriangleMesh ( mesh_name, *mesh, false, true );
}

void MakePoseGraphForFragment (
  std::string path,
  int sid, int eid,
  std::vector<std::string> colorFiles,
  std::vector<std::string> depthFiles,
  int fragment_id, int fragmentCount,
  PinholeCameraIntrinsic & intrinsic )
{
  auto fragment_posegraph_name = Format ( path + template_fragment_posegraph, fragment_id );

  // don't neet to recreate if we already did the work
  if (std::experimental::filesystem::exists ( fragment_posegraph_name ))
    return;

  PoseGraph pose_graph;
  Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity ();
  Eigen::Matrix4d trans_odometry_inv = Eigen::Matrix4d::Identity ();
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity ();
  Eigen::Matrix6d info = Eigen::Matrix6d::Identity ();
  bool success;

  pose_graph.nodes_.push_back ( PoseGraphNode ( trans_odometry ) );

  for (int s = sid; s <= eid; s++)
  {
    for (int t = s + 1; t <= eid; t++)
    {
      // odometry
      if (t == s + 1)
      {
        PrintInfo ( "Fragment %03d / %03d :: RGBD matching between frame : %d and %d\n", fragment_id, fragmentCount - 1, s, t );

        std::tie ( success, trans, info ) = RegisterOneRGBDPair ( s, t, colorFiles, depthFiles, intrinsic );

        trans_odometry = trans * trans_odometry;
        trans_odometry_inv = trans_odometry.inverse ();

        pose_graph.nodes_.push_back ( PoseGraphNode ( trans_odometry_inv ) );
        pose_graph.edges_.push_back ( PoseGraphEdge ( s - sid, t - sid, trans, info, false ) );
      }

      // keyframe loop closure
      if (s % n_keyframes_per_n_frame == 0 && t % n_keyframes_per_n_frame == 0)
      {
        PrintInfo ( "Fragment %03d / %03d :: RGBD matching between frame : %d and %d\n", fragment_id, fragmentCount - 1, s, t );

        std::tie ( success, trans, info ) = RegisterOneRGBDPair ( s, t, colorFiles, depthFiles, intrinsic );

        if (success)
        {
          pose_graph.edges_.push_back ( PoseGraphEdge ( s - sid, t - sid, trans, info, true ) );
        }
      }

    }
  }

  WritePoseGraph ( fragment_posegraph_name, pose_graph );

}

void MakeFragments::Run ()
{
  std::vector<std::string> colorFiles, depthFiles;

  auto intrinsic = PinholeCameraIntrinsic ( PinholeCameraIntrinsicParameters::PrimeSenseDefault );  

  std::string path = FullPath("..\\TestImages\\");

  MakeFolder ( path + folder_fragment );

  std::tie ( colorFiles, depthFiles ) = ReadRGBDColorFiles ( path );

  int fragmentCount = (int)std::ceil ( (float)colorFiles.size () / (float)n_frames_per_fragment );

  auto optimize = OptimizePoseGraph ();

  int sid, eid;
  for (int fragment_id = 0; fragment_id < fragmentCount; fragment_id++)
  {
    sid = fragment_id * n_frames_per_fragment;
    eid = (int)std::min ( (float)(sid + n_frames_per_fragment), (float)colorFiles.size () );

    MakePoseGraphForFragment ( path, sid, eid, colorFiles, depthFiles, fragment_id, fragmentCount, intrinsic );

    optimize.OptimizePoseGraphForFragment ( path, fragment_id );

    MakeMeshForFragment ( path, colorFiles, depthFiles, fragment_id, fragmentCount, intrinsic );

    break;
    
  }
}




