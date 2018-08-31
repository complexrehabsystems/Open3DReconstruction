#include "Helpers.h"
#include "TestReconstruction.h"
#include <Windows.h>
#include <iostream>
#include <filesystem>
#include <string>
#include <tuple>
#include <cstdio>
#include <vector>

#include <Open3D/Core/Core.h>
#include <Open3D/Core/Registration/FastGlobalRegistration.h>
#include <Open3D/IO/IO.h>
#include <Open3D/Visualization/Visualization.h>

using namespace open3d;
namespace fs = std::experimental::filesystem;

TestReconstruction::TestReconstruction ()
{
  DebugOut ( "Open3D %s", OPEN3D_VERSION );
}


TestReconstruction::~TestReconstruction ()
{

}

void TestReconstruction::Run ()
{
   

  //OdometryOption option;

  //double voxel_size = 0.05;

  //Eigen::Matrix4d odo_init = Eigen::Matrix4d::Identity ();
  //Eigen::Matrix4d trans_odo = Eigen::Matrix4d::Identity ();
  //Eigen::Matrix6d info_odo = Eigen::Matrix6d::Zero ();
  //RGBDOdometryJacobianFromHybridTerm jacobian_method;

  //ScalableTSDFVolume volume (3.0 / 512.0, 0.04, TSDFVolumeColorType::RGB8 );

  //FastGlobalRegistrationOption fastGlobalRegistrationOption ( 1.4, false, voxel_size * 0.5);

  //PoseGraph pose_graph;

  //Image color_s;
  //bool is_success;

  //for (int i = 0; i < colorFiles.size (), i < 10; i++)
  //{
  //  //auto img = depthFiles[i];

  //  //auto result = ReadImageFromPNG ( img, color_s );

  //  

  //  auto color_ss = CreateImageFromFile ( colorFiles[i] );
  //  auto depth_s = CreateImageFromFile ( depthFiles[i] );
  //  auto color_t = CreateImageFromFile ( colorFiles[i + 1] );
  //  auto depth_t = CreateImageFromFile ( depthFiles[i + 1] );

  //  auto rgbd_s = CreateRGBDImageFromColorAndDepth ( *color_ss, *depth_s );
  //  auto rgbd_t = CreateRGBDImageFromColorAndDepth ( *color_t, *depth_t );

  //  std::tie ( is_success, trans_odo, info_odo ) = ComputeRGBDOdometry ( *rgbd_s, *rgbd_t, intrinsic, odo_init, jacobian_method, option );

  //  if (is_success)
  //  {
  //    pose_graph.nodes_.push_back ( PoseGraphNode ( trans_odo ) );
  //  }

  //  DebugOut ( "ComputeRGBDOdometry success: %d - %s - %s", is_success, trans_odo, info_odo );
  //  std::cout << "Estimated 4x4 motion matrix : " << std::endl;
  //  std::cout << trans_odo << std::endl;
  //  std::cout << "Estimated 6x6 information matrix : " << std::endl;
  //  std::cout << info_odo << std::endl;


  //  auto source = CreatePointCloudFromRGBDImage ( *rgbd_s, intrinsic );
  //  auto target = CreatePointCloudFromRGBDImage ( *rgbd_t, intrinsic );

  //  auto source_down = VoxelDownSample ( *source, voxel_size );
  //  auto target_down = VoxelDownSample ( *target, voxel_size );

  //  EstimateNormals ( *source_down, KDTreeSearchParamHybrid ( 0.1, 30 ) );
  //  EstimateNormals ( *target_down, KDTreeSearchParamHybrid ( 0.1, 30 ) );

  //  std::shared_ptr<Feature> source_fpfh, target_fpfh;

  //  {
  //    ScopeTimer timer ( "source FPFH estimation with Radius 0.25" );
  //    source_fpfh = ComputeFPFHFeature ( *source_down, KDTreeSearchParamHybrid ( 0.25, 100 ) );
  //  }

  //  {
  //    ScopeTimer timer ( "target FPFH estimation with Radius 0.25" );
  //    target_fpfh = ComputeFPFHFeature ( *source_down, KDTreeSearchParamHybrid ( 0.25, 100 ) );
  //  }

  //  // global registration 
  //  RegistrationResult regResult = FastGlobalRegistration ( *source, *target, *source_fpfh, *target_fpfh, fastGlobalRegistrationOption );

  //  // local registration
  //  RegistrationResult regResult2 = RegistrationICP ( *source_down, *target_down, 0.02, trans_odo, TransformationEstimationPointToPoint ( false ) );

  //}

}


