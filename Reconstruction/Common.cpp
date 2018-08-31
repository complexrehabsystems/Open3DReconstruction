#include "Common.h"
#include <Windows.h>

std::vector<std::string> GetFileList ( std::string path, std::string extension)
{
  std::vector<std::string> files;

  for (auto & p : std::experimental::filesystem::directory_iterator ( path ))
  {
    if (!extension.empty () && extension != p.path ().extension ().string ())
      continue;

    files.push_back ( p.path ().string () );
  }

  // note: assuming already sorted alpha numerically

  return files;
}

std::tuple<std::vector<std::string>, std::vector<std::string>> ReadRGBDColorFiles ( std::string path )
{
  std::vector<std::string> colorFiles, depthFiles;

  colorFiles = GetFileList ( path + "rgb" );
  depthFiles = GetFileList ( path + "depth" );

  return std::make_tuple ( colorFiles, depthFiles );
}

void MakeFolder ( std::string path_folder )
{
  CreateDirectoryA ( path_folder.c_str (), nullptr );
}

std::string FullPath ( std::string path )
{
  char fullPath[MAX_PATH];

  GetFullPathNameA ( path.c_str (), MAX_PATH, fullPath, NULL );

  return fullPath;
}