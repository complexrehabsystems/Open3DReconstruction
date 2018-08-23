#include "stdafx.h"
#include "Helpers.h"
#include "TestReconstruction.h"
#include <cstdio>
#include <vector>

#include <Open3D/Core/Core.h>
#include <Open3D/IO/IO.h>
#include <Open3D/Visualization/Visualization.h>

#include <string>
#include <iostream>
#include <filesystem>

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

  std::string chairPath = "..\\TestImages\\";

  std::vector<std::string> colorFiles, depthFiles;

  fs::directory_iterator end_itr; // default construction yields past-the-end

  for (auto & p : fs::directory_iterator ( chairPath + "rgb" ))
  {
    colorFiles.push_back ( p.path ().string () );
  }

  for (auto & p : fs::directory_iterator ( chairPath + "depth" ))
  {
    depthFiles.push_back ( p.path ().string () );
  }

  auto intrinsic = PinholeCameraIntrinsic ( PinholeCameraIntrinsicParameters::PrimeSenseDefault );

  OdometryOption option;

  Eigen::Matrix4d odo_init = Eigen::Matrix4d::Identity ();
  Eigen::Matrix4d trans_odo = Eigen::Matrix4d::Identity ();
  Eigen::Matrix6d info_odo = Eigen::Matrix6d::Zero ();
  RGBDOdometryJacobianFromHybridTerm jacobian_method;

  Image color_s;
  bool is_success;

  for (int i = 0; i < colorFiles.size (), i < 10; i++)
  {
    auto img = depthFiles[i];

    auto result = ReadImageFromPNG ( img, color_s );

    auto color_ss = CreateImageFromFile ( colorFiles[i] );
    auto depth_s = CreateImageFromFile ( depthFiles[i] );
    auto color_t = CreateImageFromFile ( colorFiles[i + 1] );
    auto depth_t = CreateImageFromFile ( depthFiles[i + 1] );

    auto rgbd_s = CreateRGBDImageFromColorAndDepth ( *color_ss, *depth_s );
    auto rgbd_t = CreateRGBDImageFromColorAndDepth ( *color_t, *depth_t );

    /*std::tie ( is_success, trans_odo, info_odo ) = ComputeRGBDOdometry ( *rgbd_s, *rgbd_t, intrinsic, odo_init, jacobian_method, option );

    std::cout << "Estimated 4x4 motion matrix : " << std::endl;
    std::cout << trans_odo << std::endl;
    std::cout << "Estimated 6x6 information matrix : " << std::endl;
    std::cout << info_odo << std::endl;*/

    auto source = CreatePointCloudFromRGBDImage ( *rgbd_s, intrinsic );
    auto target = CreatePointCloudFromRGBDImage ( *rgbd_t, intrinsic );

    auto source_down = VoxelDownSample ( *source, 0.05f );

    EstimateNormals ( *source_down, KDTreeSearchParamHybrid ( 0.1, 30 ) );
    {
      ScopeTimer timer ( "FPFH estimation with Radius 0.25" );
      auto pcd_fpfh = ComputeFPFHFeature ( *source_down, KDTreeSearchParamHybrid ( 0.25, 100 ) );
    }

  }

}
