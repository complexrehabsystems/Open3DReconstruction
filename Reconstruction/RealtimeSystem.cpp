#include "RealtimeSystem.h"
#include "Helpers.h"
#include "Common.h"
#include "Open3dCommon.h"

#include <Open3D/Core/Core.h>
#include <Open3D/Core/Registration/FastGlobalRegistration.h>
#include <Open3D/IO/IO.h>
#include <Open3D/Visualization/Visualization.h>


#include <iostream>
#include <tuple>
#include <cstdio>

using namespace open3d;


RealtimeSystem::RealtimeSystem ()
{
}


RealtimeSystem::~RealtimeSystem ()
{
}

void RealtimeSystem::Run ( std::string path, open3d::PinholeCameraIntrinsic intrinsic )
{
  std::vector<std::string> colorFiles, depthFiles;

  std::tie ( colorFiles, depthFiles ) = ReadRGBDColorFiles ( path );

  std::shared_ptr<Image> color_s, depth_s, color_t, depth_t;
  std::shared_ptr<RGBDImage> rgbd_s, rgbd_t;
  std::shared_ptr<PointCloud> pcd_s, pcd_t, pcd_down_s, pcd_down_t;
  std::shared_ptr<Feature> fpfh_s, fpfh_t;
  std::shared_ptr<RegistrationResult> registration_result_ptr;
  RegistrationResult registration_result;
  Eigen::Matrix6d information;

  std::vector<std::reference_wrapper<const CorrespondenceChecker>>
    correspondence_checker;
  auto correspondence_checker_edge_length =
    CorrespondenceCheckerBasedOnEdgeLength ( 0.9 );
  auto correspondence_checker_distance =
    CorrespondenceCheckerBasedOnDistance ( 0.075 );
  auto correspondence_checker_normal =
    CorrespondenceCheckerBasedOnNormal ( 0.52359878 );

  correspondence_checker.push_back ( correspondence_checker_edge_length );
  correspondence_checker.push_back ( correspondence_checker_distance );
  correspondence_checker.push_back ( correspondence_checker_normal );

  registration_result.transformation_ = Eigen::Matrix4d::Identity ();

  ScalableTSDFVolume volume ( 3.0 / 512.0, 0.04, TSDFVolumeColorType::RGB8 );

  int fragment_size = 100;

  for (int i = 0; i < colorFiles.size (); i++)
  {
    color_t = CreateImageFromFile ( colorFiles[i] );
    depth_t = CreateImageFromFile ( depthFiles[i] );
    rgbd_t = CreateRGBDImageFromColorAndDepth ( *color_t, *depth_t, 1000.0, 3.0, false );
    pcd_t = CreatePointCloudFromRGBDImage ( *rgbd_t, intrinsic );
    std::tie ( pcd_down_t, fpfh_t ) = PreprocessPointCloud ( *pcd_t );

    if (i != 0)
    {
      /* {
     ScopeTimer t ( "RegisterColoredPointCloudICP" );
     std::tie( registration_result_ptr, information) = RegisterColoredPointCloudICP (
       *pcd_s, *pcd_t );

     registration_result = *registration_result_ptr;

     PrintInfo ( "RegisterColoredPointCloudICP rmse: %f fitness: %f\n", registration_result.inlier_rmse_, registration_result.fitness_ );
   }
   VisualizeRegistration ( *pcd_s, *pcd_t, registration_result.transformation_ );*/

      {
        ScopeTimer t ( "RegistrationRANSACBasedOnFeatureMatching" );
        registration_result = RegistrationRANSACBasedOnFeatureMatching (
          *pcd_down_s, *pcd_down_t, *fpfh_s, *fpfh_t, 0.075,
          TransformationEstimationPointToPoint ( false ), 4,
          correspondence_checker, RANSACConvergenceCriteria ( 4000000, 1000 ) );

        PrintInfo ( "RegistrationRANSACBasedOnFeatureMatching rmse: %f fitness: %f\n", registration_result.inlier_rmse_, registration_result.fitness_ );
      }
      //VisualizeRegistration ( *pcd_s, *pcd_t, registration_result.transformation_ );

      /*{
        ScopeTimer t ( "FastGlobalRegistration" );
        registration_result = FastGlobalRegistration ( *pcd_down_s, *pcd_down_t, *fpfh_s, *fpfh_t, FastGlobalRegistrationOption ( 1.4, false, true, 0.07 ) );

        PrintInfo ( "FastGlobalRegistration rmse: %f fitness: %f\n", registration_result.inlier_rmse_, registration_result.fitness_ );
      }
      VisualizeRegistration ( *pcd_s, *pcd_t, registration_result.transformation_ );*/
    }

    volume.Integrate ( *rgbd_t, intrinsic, registration_result.transformation_ );

    if (i % fragment_size == 0)
    {
      auto mesh = volume.ExtractTriangleMesh ();
      mesh->ComputeVertexNormals ();

      VisualizeRegistration ( *mesh, registration_result.transformation_ );
    }

    color_s = color_t;
    depth_s = depth_t;
    rgbd_s = rgbd_t;
    pcd_s = pcd_t;
    pcd_down_s = pcd_down_t;
    fpfh_s = fpfh_t;
  }
}
