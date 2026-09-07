#pragma once

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace explorer_bridge
{

constexpr int8_t kUnknown = -1;
constexpr int8_t kFree = 0;
constexpr int8_t kOccupied = 100;

class OccupancyMap
{
public:
  OccupancyMap(double resolution, double initial_size_m);

  double resolution() const {return resolution_;}
  double originX() const {return origin_x_;}
  double originY() const {return origin_y_;}
  int height() const {return static_cast<int>(data_.size() / static_cast<size_t>(width_));}
  int width() const {return width_;}
  const std::vector<int8_t> & data() const {return data_;}
  std::vector<int8_t> & data() {return data_;}

  int8_t at(int row, int col) const;
  int8_t & at(int row, int col);

  std::pair<int, int> worldToCell(double x, double y) const;
  void ensureContains(double x, double y, double margin_m = 1.0);

private:
  void expand();

  double resolution_;
  double origin_x_;
  double origin_y_;
  int width_;
  std::vector<int8_t> data_;
};

}  // namespace explorer_bridge
