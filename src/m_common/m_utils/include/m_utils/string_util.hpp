#pragma once

#include <string>
#include <vector>

namespace m_utils {

std::vector<std::string> split(const std::string& s, char delim);

std::string trim(const std::string& s);

}  // namespace m_utils
