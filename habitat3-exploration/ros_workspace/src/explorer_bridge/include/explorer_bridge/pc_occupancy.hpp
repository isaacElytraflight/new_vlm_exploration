#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

#include "explorer_bridge/occupancy_map.hpp"

namespace explorer_bridge
{

constexpr double kDefaultCameraZ_m = 0.1;
constexpr double kDefaultSensorFar_m = 50.0;
constexpr double kDefaultSatEps_m = 0.5;
constexpr double kDefaultWallHeightMin_m = 0.05;
constexpr double kDefaultWallHeightMax_m = 1.0;

struct DepthIntrinsics
{
  double fx{320.0};
  double fy{320.0};
  double cx{320.0};
  double cy{240.0};
};

struct IntegrateDepthParams
{
  double robot_x{0.0};
  double robot_y{0.0};
  double yaw{0.0};
  DepthIntrinsics K{};
  double range_min{0.1};
  double range_max{10.0};
  double camera_z{kDefaultCameraZ_m};
  double sensor_far{kDefaultSensorFar_m};
  double sat_eps{kDefaultSatEps_m};
  double wall_height_min{kDefaultWallHeightMin_m};
  double wall_height_max{kDefaultWallHeightMax_m};
  int subsample{8};
};

inline bool isWallHeight(double z_m, double min_z, double max_z)
{
  return z_m >= min_z && z_m <= max_z;
}

std::optional<std::tuple<double, double, double>> pixelToBaseLinkXyz(
  int col, int row, float depth,
  const DepthIntrinsics & K, double camera_z);

std::pair<double, double> baseLinkXyToMap(
  double x_bl, double y_bl,
  double robot_x, double robot_y, double yaw);

double normalizeRange(
  double raw, double range_min, double clear_range,
  double sensor_far, double sat_eps);

/// In-place Bresenham carve; stops at first existing OCCUPIED cell.
void carveRayInPlace(
  OccupancyMap & grid,
  int r0, int c0, int r1, int c1,
  bool mark_occ);

/// Ray-carve free space from a row-major depth image (height * width floats).
void integrateDepthFrame(
  OccupancyMap & grid,
  const float * depth,
  int height,
  int width,
  const IntegrateDepthParams & params);

/// Inflate OCCUPIED cells by radius (publish-time copy).
std::vector<int8_t> inflateOccupied(
  const OccupancyMap & grid, int radius_cells);

int inflationRadiusCells(double resolution, double radius_m);

}  // namespace explorer_bridge
