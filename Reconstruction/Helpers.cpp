#include "Helpers.h"

#include <windows.h>

std::wstring s2ws (const std::string& s)
{
  int len;
  int slength = (int)s.length () + 1;
  len = MultiByteToWideChar (CP_ACP, 0, s.c_str (), slength, 0, 0);
  wchar_t* buf = new wchar_t[len];
  MultiByteToWideChar (CP_ACP, 0, s.c_str (), slength, buf, len);
  std::wstring r (buf);
  delete[] buf;
  return r;
}


