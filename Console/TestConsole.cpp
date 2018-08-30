#include "Common.h"
#include "MakeFragments.h"
#include "RegisterFragments.h"
#include "IntegrateScene.h"

#include <Open3D/Core/Core.h>
#include <iostream>

using namespace open3d;

int main(int argc, char *argv[])
{

  auto intrinsic = PinholeCameraIntrinsic ( PinholeCameraIntrinsicParameters::PrimeSenseDefault );

  std::string path = FullPath ( "..\\TestImages\\" );

  {
    ScopeTimer timer ( "Full reconstruction" );
    
    std::cout << "start MakeFragments" << std::endl;
    auto frag = MakeFragments ();
    frag.Run ( path, intrinsic );
    std::cout << "end MakeFragments" << std::endl;    

    std::cout << "start RegisterFragments" << std::endl;
    auto reg = RegisterFragments ();
    reg.Run ( path );
    std::cout << "end RegisterFragments" << std::endl;    

    std::cout << "start IntegrateScene" << std::endl;
    auto integration = IntegrateScene ();
    integration.Run ( path, intrinsic );
    std::cout << "end RegisterFragments" << std::endl;
  }
}
