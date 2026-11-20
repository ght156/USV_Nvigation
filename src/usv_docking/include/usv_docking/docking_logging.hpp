#ifndef USV_DOCKING__DOCKING_LOGGING_HPP_
#define USV_DOCKING__DOCKING_LOGGING_HPP_

#include <mlogger/mlogger.hpp>

#include <cstdlib>
#include <string>

namespace usv_docking::docking_logging
{

inline std::string log_root()
{
  const char * home = std::getenv("USV_HOME");
  return home && *home ? std::string(home) + "/logs" : "logs";
}

}  // namespace usv_docking::docking_logging

#endif  // USV_DOCKING__DOCKING_LOGGING_HPP_
