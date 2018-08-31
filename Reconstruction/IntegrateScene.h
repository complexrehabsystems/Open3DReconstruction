#pragma once

#include <string>

namespace open3d
{
  class PinholeCameraIntrinsic;
}

class IntegrateScene
{
public:
  IntegrateScene ();
  ~IntegrateScene ();

  void Run ( std::string path, open3d::PinholeCameraIntrinsic intrinsic );
};

