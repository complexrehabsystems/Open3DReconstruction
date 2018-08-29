#include "OptimizePoseGraph.h"
#include "Helpers.h"
#include "Common.h"

#include <Open3D/Core/Core.h>
#include <Open3D/Core/Registration/GlobalOptimization.h>
#include <Open3D/IO/IO.h>

#include <string>

using namespace open3d;

OptimizePoseGraph::OptimizePoseGraph ()
{
}


OptimizePoseGraph::~OptimizePoseGraph ()
{
}

void RunPoseGraphOptimization (
  std::string pose_graph_name,
  std::string pose_graph_optimized_name,
  double max_correspondence_distance,
  double preference_loop_closure)
{
  auto method = GlobalOptimizationLevenbergMarquardt ();
  auto criteria = GlobalOptimizationConvergenceCriteria ();
  auto option = GlobalOptimizationOption ( max_correspondence_distance, 0.25, preference_loop_closure, 0 );

  PoseGraph pose_graph;
  ReadPoseGraph ( pose_graph_name, pose_graph );
  GlobalOptimization ( pose_graph, method, criteria, option );
  WritePoseGraph ( pose_graph_optimized_name, pose_graph );

}

void OptimizePoseGraph::OptimizePoseGraphForFragment ( std::string path, int fragment_id )
{
  auto pose_graph_name = Format ( path + template_fragment_posegraph, fragment_id );
  auto pose_graph_name_optimized = path + template_fragment_posegraph_optimized;

  RunPoseGraphOptimization ( pose_graph_name, pose_graph_name_optimized, 0.07, 0.1 );
}
