#pragma once

#include <string>

class OptimizePoseGraph
{
public:
  OptimizePoseGraph ();
  ~OptimizePoseGraph ();

  void OptimizePoseGraphForFragment ( std::string path, int fragment_id );
};

