#include "Common.h"
#include "MakeFragments.h"
#include "RegisterFragments.h"

#include <Open3D/Core/Core.h>
#include <iostream>

using namespace open3d;

int main(int argc, char *argv[])
{

  auto intrinsic = PinholeCameraIntrinsic ( PinholeCameraIntrinsicParameters::PrimeSenseDefault );

  std::string path = FullPath ( "..\\TestImages\\" );

  auto frag = MakeFragments ();

  std::cout << "start MakeFragments" << std::endl;

  frag.Run (path, intrinsic);

  std::cout << "end MakeFragments" << std::endl;

  auto reg = RegisterFragments ();

  std::cout << "start RegisterFragments" << std::endl;

  reg.Run (path);

  std::cout << "end RegisterFragments" << std::endl;
}
