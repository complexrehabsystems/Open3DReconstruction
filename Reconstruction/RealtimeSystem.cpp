#include "RealtimeSystem.h"
#include "Helpers.h"
#include "Common.h"

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
}
