#pragma once

#include <string>

namespace explorer_mission
{

/// Stub Nav2 navigator — reserved for future nav2 integration.
class Nav2Navigator
{
public:
  Nav2Navigator() = default;

  bool navigateToPose(double /*x*/, double /*y*/, double /*yaw_deg*/)
  {
    return false;
  }

  void cancel() {}

  std::string lastError() const {return "Nav2Navigator is a stub; use DiscreteMove instead.";}
};

}  // namespace explorer_mission
