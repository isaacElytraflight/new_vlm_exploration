#include "explorer_bridge/occupancy_map.hpp"

#include <algorithm>
#include <stdexcept>

namespace explorer_bridge
{

OccupancyMap::OccupancyMap(double resolution, double initial_size_m)
: resolution_(resolution),
  width_(0)
{
  if (resolution_ <= 0.0) {
    throw std::invalid_argument("resolution must be positive");
  }
  const int cells = std::max(2, static_cast<int>(std::ceil(initial_size_m / resolution_)));
  width_ = cells;
  data_.assign(static_cast<size_t>(cells) * static_cast<size_t>(cells), kUnknown);
  const double half = (cells * resolution_) / 2.0;
  origin_x_ = -half;
  origin_y_ = -half;
}

int8_t OccupancyMap::at(int row, int col) const
{
  return data_[static_cast<size_t>(row) * static_cast<size_t>(width_) + static_cast<size_t>(col)];
}

int8_t & OccupancyMap::at(int row, int col)
{
  return data_[static_cast<size_t>(row) * static_cast<size_t>(width_) + static_cast<size_t>(col)];
}

std::pair<int, int> OccupancyMap::worldToCell(double x, double y) const
{
  const int col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return {row, col};
}

void OccupancyMap::ensureContains(double x, double y, double margin_m)
{
  const double min_x = x - margin_m;
  const double max_x = x + margin_m;
  const double min_y = y - margin_m;
  const double max_y = y + margin_m;
  while (true) {
    const auto [r0, c0] = worldToCell(min_x, min_y);
    const auto [r1, c1] = worldToCell(max_x, max_y);
    if (0 <= r0 && r0 < height() && 0 <= c0 && c0 < width_ &&
      0 <= r1 && r1 < height() && 0 <= c1 && c1 < width_)
    {
      return;
    }
    expand();
  }
}

void OccupancyMap::expand()
{
  const int old_h = height();
  const int old_w = width_;
  const int new_h = old_h * 2;
  const int new_w = old_w * 2;
  std::vector<int8_t> next(static_cast<size_t>(new_h) * static_cast<size_t>(new_w), kUnknown);
  const int r0 = (new_h - old_h) / 2;
  const int c0 = (new_w - old_w) / 2;
  for (int r = 0; r < old_h; ++r) {
    for (int c = 0; c < old_w; ++c) {
      next[static_cast<size_t>(r0 + r) * static_cast<size_t>(new_w) + static_cast<size_t>(c0 + c)] =
        at(r, c);
    }
  }
  origin_x_ -= c0 * resolution_;
  origin_y_ -= r0 * resolution_;
  width_ = new_w;
  data_.swap(next);
}

}  // namespace explorer_bridge
