#include "MakeFragments.h"
#include "RegisterFragments.h"

#include <iostream>

int main(int argc, char *argv[])
{
  /*auto frag = MakeFragments ();

  std::cout << "start MakeFragments" << std::endl;

  frag.Run ();

  std::cout << "end MakeFragments" << std::endl;*/

  auto reg = RegisterFragments ();

  std::cout << "start RegisterFragments" << std::endl;

  reg.Run ();

  std::cout << "end RegisterFragments" << std::endl;
}
