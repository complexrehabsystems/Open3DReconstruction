#pragma once

#include <string>
#include <filesystem>
#include <vector>


extern int n_frames_per_fragment;
extern int n_keyframes_per_n_frame;

extern std::string folder_fragment;
extern std::string template_fragment_posegraph;
extern std::string template_fragment_posegraph_optimized;
extern std::string template_fragment_mesh;

extern std::string folder_scene;
extern std::string template_global_posegraph;
extern std::string template_global_posegraph_optimized;
extern std::string template_global_mesh;


std::vector<std::string> GetFileList ( std::string path, std::string extension = "" );

void MakeFolder ( std::string path_folder );

std::string FullPath ( std::string path );

