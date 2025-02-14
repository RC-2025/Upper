#pragma once
#include <iostream>
#include <rc_localization/basic_debugger.hpp>
#include <vector>
template <typename T>
std::ostream &operator<<(std::ostream &os, std::vector<T> &__vec) {
  // os << "size:" << __vec.size();
  os << '[';

  for (auto &i : __vec)
    os << i << ' ';

  os << ']';
  return os;
}

#define GET_PARAM_DEBUG(name, param)                                           \
  this->get_parameter(name, param);                                            \
  ROBOT_INFO_STRAM(name << ":\n" << param);
