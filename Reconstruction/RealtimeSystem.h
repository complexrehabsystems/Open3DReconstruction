#pragma once

#include <string>

namespace open3d
{
  class PinholeCameraIntrinsic;
}

class RealtimeSystem
{
public:
  RealtimeSystem ();
  ~RealtimeSystem ();

  void Run ( std::string path, open3d::PinholeCameraIntrinsic intrinsic );
};

