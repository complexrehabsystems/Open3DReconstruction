#pragma once

#include <string>

namespace open3d
{
  class PinholeCameraIntrinsic;
}

class MakeFragments
{
public:
  MakeFragments ();
  ~MakeFragments ();

  void Run (std::string path, open3d::PinholeCameraIntrinsic intrinsic);
private:
  

};

