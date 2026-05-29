#include "m_utils/string_util.hpp"

#include <sstream>

namespace m_utils {

std::vector<std::string> split(const std::string& s, char delim)
{
  std::vector<std::string> out;
  std::stringstream        ss(s);
  std::string              item;
  while (std::getline(ss, item, delim)) {
    out.push_back(item);
  }
  return out;
}

std::string trim(const std::string& s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

}  // namespace m_utils
