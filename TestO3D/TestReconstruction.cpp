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

TestReconstruction::TestReconstruction ()
{
  DebugOut ( "Open3D %s", OPEN3D_VERSION );
}


TestReconstruction::~TestReconstruction ()
{
}

void TestReconstruction::Run ()
{
  

  OdometryOption option;
}
