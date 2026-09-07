#include "explorer_bridge/pc_occupancy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace explorer_bridge
{

std::optional<std::tuple<double, double, double>> pixelToBaseLinkXyz(
  int col, int row, float depth,
  const DepthIntrinsics & K, double camera_z)
{
  if (!std::isfinite(depth) || depth <= 0.0f) {
    return std::nullopt;
  }
  const double z_c = static_cast<double>(depth);
  const double x_c = (static_cast<double>(col) - K.cx) / K.fx * z_c;
  const double y_c = (static_cast<double>(row) - K.cy) / K.fy * z_c;
  const double x_bl = z_c;
  const double y_bl = -x_c;
  const double z_bl = -y_c + camera_z;
  if (x_bl <= 0.01) {
    return std::nullopt;
  }
  return std::make_tuple(x_bl, y_bl, z_bl);
}

std::pair<double, double> baseLinkXyToMap(
  double x_bl, double y_bl,
  double robot_x, double robot_y, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {
    robot_x + x_bl * c - y_bl * s,
    robot_y + x_bl * s + y_bl * c,
  };
}

double normalizeRange(
  double raw, double range_min, double clear_range,
  double sensor_far, double sat_eps)
{
  if (!std::isfinite(raw)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (raw < range_min) {
    return clear_range;
  }
  const double far = std::max(sensor_far, clear_range);
  if (raw >= far - std::max(0.0, sat_eps)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (raw > clear_range) {
    return clear_range;
  }
  return raw;
}

void carveRayInPlace(
  OccupancyMap & grid,
  int r0, int c0, int r1, int c1,
  bool mark_occ)
{
  const int h = grid.height();
  const int w = grid.width();
  const int dr = std::abs(r1 - r0);
  const int dc = std::abs(c1 - c0);
  const int sr = (r0 < r1) ? 1 : -1;
  const int sc = (c0 < c1) ? 1 : -1;
  int err = dr - dc;
  int r = r0;
  int c = c0;

  while (true) {
    if (0 <= r && r < h && 0 <= c && c < w) {
      int8_t & cell = grid.at(r, c);
      const bool at_end = (r == r1 && c == c1);
      if (at_end && mark_occ) {
        cell = kOccupied;
      } else if (cell == kOccupied) {
        // Early stop: do not paint free past known walls.
        break;
      } else {
        cell = kFree;
      }
    }

    if (r == r1 && c == c1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 > -dc) {
      err -= dc;
      r += sr;
    }
    if (e2 < dr) {
      err += dr;
      c += sc;
    }
  }
}

void integrateDepthFrame(
  OccupancyMap & grid,
  const float * depth,
  int height,
  int width,
  const IntegrateDepthParams & params)
{
  if (depth == nullptr || height <= 0 || width <= 0) {
    throw std::invalid_argument("depth must be non-null 2-D");
  }

  const double free_eps = std::max(grid.resolution() * 0.5, 1e-3);
  const int step = std::max(1, params.subsample);

  grid.ensureContains(
    params.robot_x, params.robot_y,
    std::max(grid.resolution() * 4.0, 0.2));

  struct Prepared
  {
    double end_x;
    double end_y;
    bool mark_occ;
  };
  std::vector<Prepared> prepared;
  prepared.reserve(static_cast<size_t>((height / step + 1) * (width / step + 1)));

  for (int row = 0; row < height; row += step) {
    for (int col = 0; col < width; col += step) {
      const float raw_depth = depth[static_cast<size_t>(row) * static_cast<size_t>(width) +
        static_cast<size_t>(col)];
      const auto xyz = pixelToBaseLinkXyz(col, row, raw_depth, params.K, params.camera_z);
      if (!xyz.has_value()) {
        continue;
      }
      const double x_bl = std::get<0>(*xyz);
      const double y_bl = std::get<1>(*xyz);
      const double z_bl = std::get<2>(*xyz);
      const double horiz = std::hypot(x_bl, y_bl);
      const double classified = normalizeRange(
        horiz, params.range_min, params.range_max, params.sensor_far, params.sat_eps);
      if (!std::isfinite(classified)) {
        continue;
      }
      const bool is_hit = classified < (params.range_max - free_eps);
      const double use_range = std::min(classified, params.range_max);
      const double scale = (horiz > 1e-6) ? (use_range / horiz) : 0.0;
      const double end_x_bl = x_bl * scale;
      const double end_y_bl = y_bl * scale;
      const double end_z_bl = z_bl * scale;
      const auto [end_x, end_y] = baseLinkXyToMap(
        end_x_bl, end_y_bl, params.robot_x, params.robot_y, params.yaw);
      grid.ensureContains(end_x, end_y, grid.resolution() * 2.0);
      const bool mark_occ = is_hit && isWallHeight(
        end_z_bl, params.wall_height_min, params.wall_height_max);
      prepared.push_back({end_x, end_y, mark_occ});
    }
  }

  // Recompute robot cell after possible expands during prepare.
  const auto [rr2, rc2] = grid.worldToCell(params.robot_x, params.robot_y);

  for (const auto & p : prepared) {
    if (!p.mark_occ) {
      continue;
    }
    const auto [er, ec] = grid.worldToCell(p.end_x, p.end_y);
    if (0 <= er && er < grid.height() && 0 <= ec && ec < grid.width()) {
      grid.at(er, ec) = kOccupied;
    }
  }

  for (const auto & p : prepared) {
    const auto [er, ec] = grid.worldToCell(p.end_x, p.end_y);
    carveRayInPlace(grid, rr2, rc2, er, ec, p.mark_occ);
  }
}

std::vector<int8_t> inflateOccupied(const OccupancyMap & grid, int radius_cells)
{
  std::vector<int8_t> out = grid.data();
  if (radius_cells <= 0) {
    return out;
  }
  const int h = grid.height();
  const int w = grid.width();
  const int r = radius_cells;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (grid.at(y, x) != kOccupied) {
        continue;
      }
      const int y0 = std::max(0, y - r);
      const int y1 = std::min(h, y + r + 1);
      const int x0 = std::max(0, x - r);
      const int x1 = std::min(w, x + r + 1);
      for (int yy = y0; yy < y1; ++yy) {
        for (int xx = x0; xx < x1; ++xx) {
          out[static_cast<size_t>(yy) * static_cast<size_t>(w) + static_cast<size_t>(xx)] =
            kOccupied;
        }
      }
    }
  }
  return out;
}

int inflationRadiusCells(double resolution, double radius_m)
{
  if (radius_m <= 0.0 || resolution <= 0.0) {
    return 0;
  }
  return static_cast<int>(std::ceil(radius_m / resolution));
}

}  // namespace explorer_bridge
