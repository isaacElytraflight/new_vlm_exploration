#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "explorer_bridge/occupancy_map.hpp"
#include "explorer_bridge/pc_occupancy.hpp"

namespace
{

constexpr double kFx = 320.0;
constexpr double kFy = 320.0;
constexpr double kCx = 320.0;
constexpr double kCy = 240.0;

explorer_bridge::IntegrateDepthParams defaultParams()
{
  explorer_bridge::IntegrateDepthParams p;
  p.K = {kFx, kFy, kCx, kCy};
  p.range_min = 0.1;
  p.range_max = 10.0;
  p.sensor_far = 50.0;
  p.sat_eps = 0.5;
  p.camera_z = 0.1;
  p.wall_height_min = 0.05;
  p.wall_height_max = 1.0;
  p.subsample = 2;
  return p;
}

std::vector<float> blankDepth(int h, int w, float fill)
{
  return std::vector<float>(static_cast<size_t>(h) * static_cast<size_t>(w), fill);
}

void paintCenterBand(std::vector<float> & depth, int h, int w, float value)
{
  // Paint a horizontal band around cy so wall-height hits land near z~0.1–1.0.
  const int row0 = std::max(0, static_cast<int>(kCy) - 20);
  const int row1 = std::min(h, static_cast<int>(kCy) + 20);
  for (int r = row0; r < row1; ++r) {
    for (int c = 0; c < w; ++c) {
      depth[static_cast<size_t>(r) * static_cast<size_t>(w) + static_cast<size_t>(c)] = value;
    }
  }
}

}  // namespace

TEST(PcOccupancyHarness, RunnerExecutesAssertions)
{
  EXPECT_EQ(1 + 1, 2);
}

TEST(PcOccupancyHarness, IntentionalFailureIsDetectable_NegativeControl)
{
  EXPECT_FALSE(1 == 2);
}

TEST(PcOccupancy, WallBandHitPaintsOccupied_Positive)
{
  constexpr int H = 480;
  constexpr int W = 640;
  auto depth = blankDepth(H, W, 49.7f);
  paintCenterBand(depth, H, W, 3.0f);

  explorer_bridge::OccupancyMap grid(0.05, 24.0);
  auto params = defaultParams();
  explorer_bridge::integrateDepthFrame(grid, depth.data(), H, W, params);

  const auto [er, ec] = grid.worldToCell(3.0, 0.0);
  bool found_occ = false;
  for (int r = std::max(0, er - 5); r < std::min(grid.height(), er + 6); ++r) {
    for (int c = std::max(0, ec - 5); c < std::min(grid.width(), ec + 6); ++c) {
      if (grid.at(r, c) == explorer_bridge::kOccupied) {
        found_occ = true;
      }
    }
  }
  EXPECT_TRUE(found_occ);
}

TEST(PcOccupancy, FloorOnlyDoesNotPaintOccupied_Negative)
{
  constexpr int H = 480;
  constexpr int W = 640;
  // Floor-looking depth: near range on bottom rows → z below wall band after projection.
  auto depth = blankDepth(H, W, std::nanf(""));
  for (int r = H - 24; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      depth[static_cast<size_t>(r) * static_cast<size_t>(W) + static_cast<size_t>(c)] = 0.27f;
    }
  }

  explorer_bridge::OccupancyMap grid(0.05, 24.0);
  explorer_bridge::integrateDepthFrame(grid, depth.data(), H, W, defaultParams());

  int occ = 0;
  for (const auto v : grid.data()) {
    if (v == explorer_bridge::kOccupied) {
      ++occ;
    }
  }
  EXPECT_EQ(occ, 0);
}

TEST(PcOccupancy, FreeCarveMarksUnknownToFree_Positive)
{
  constexpr int H = 120;
  constexpr int W = 160;
  auto depth = blankDepth(H, W, 2.0f);
  // Center pixel at cy-ish for this small image
  explorer_bridge::OccupancyMap grid(0.05, 10.0);
  auto params = defaultParams();
  params.K.cx = W / 2.0;
  params.K.cy = H / 2.0;
  params.subsample = 1;
  explorer_bridge::integrateDepthFrame(grid, depth.data(), H, W, params);

  const auto [rr, rc] = grid.worldToCell(0.0, 0.0);
  const auto [er, ec] = grid.worldToCell(1.0, 0.0);
  EXPECT_EQ(grid.at(rr, rc), explorer_bridge::kFree);
  // Midway should be free if ray cleared
  const int mr = (rr + er) / 2;
  const int mc = (rc + ec) / 2;
  EXPECT_NE(grid.at(mr, mc), explorer_bridge::kUnknown);
}

TEST(PcOccupancy, FreeDoesNotClearOccupied_Negative)
{
  explorer_bridge::OccupancyMap grid(0.05, 10.0);
  grid.ensureContains(0.0, 0.0, 2.0);
  grid.ensureContains(2.0, 0.0, 2.0);
  const auto [rr, rc] = grid.worldToCell(0.0, 0.0);
  const auto [er, ec] = grid.worldToCell(2.0, 0.0);
  grid.at(er, ec) = explorer_bridge::kOccupied;

  // Carve a free (non-hit) ray to same endpoint — must not clear OCCUPIED.
  explorer_bridge::carveRayInPlace(grid, rr, rc, er, ec, false);
  EXPECT_EQ(grid.at(er, ec), explorer_bridge::kOccupied);
}

TEST(PcOccupancy, EarlyStopDoesNotPaintPastOccupied_Positive)
{
  explorer_bridge::OccupancyMap grid(0.05, 10.0);
  grid.ensureContains(0.0, 0.0, 3.0);
  grid.ensureContains(3.0, 0.0, 3.0);
  const auto [r0, c0] = grid.worldToCell(0.0, 0.0);
  const auto [rm, cm] = grid.worldToCell(1.0, 0.0);
  const auto [r1, c1] = grid.worldToCell(3.0, 0.0);
  grid.at(rm, cm) = explorer_bridge::kOccupied;

  explorer_bridge::carveRayInPlace(grid, r0, c0, r1, c1, false);

  // Beyond the occupied cell toward the far endpoint should stay UNKNOWN.
  const auto [rb, cb] = grid.worldToCell(2.0, 0.0);
  EXPECT_EQ(grid.at(rb, cb), explorer_bridge::kUnknown);
  EXPECT_EQ(grid.at(rm, cm), explorer_bridge::kOccupied);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
