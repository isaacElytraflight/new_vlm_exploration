#include "explorer_mission/discrete_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

namespace explorer_mission
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

struct LatticeKey
{
  int xi{0};
  int yi{0};
  int yaw{0};

  bool operator==(const LatticeKey & o) const
  {
    return xi == o.xi && yi == o.yi && yaw == o.yaw;
  }
};

struct LatticeKeyHash
{
  std::size_t operator()(const LatticeKey & k) const noexcept
  {
    const std::size_t a = static_cast<std::size_t>(static_cast<uint32_t>(k.xi)) * 73856093u;
    const std::size_t b = static_cast<std::size_t>(static_cast<uint32_t>(k.yi)) * 19349663u;
    const std::size_t c = static_cast<std::size_t>(static_cast<uint32_t>(k.yaw)) * 83492791u;
    return a ^ b ^ c;
  }
};

struct SearchNode
{
  LatticeKey key;
  double x{0.0};
  double y{0.0};
  double g{0.0};
  int parent{-1};
  uint8_t action{255};
};

struct OpenEntry
{
  int idx{0};
  double f{0.0};
};

struct OpenCmp
{
  bool operator()(const OpenEntry & a, const OpenEntry & b) const
  {
    return a.f > b.f;
  }
};

int yawToBin(double yaw_deg)
{
  double y = normalizeAngleDeg(yaw_deg);
  if (y < 0.0) {
    y += 360.0;
  }
  int bin = static_cast<int>(std::lround(y / TURN_DEG)) % YAW_BINS;
  if (bin < 0) {
    bin += YAW_BINS;
  }
  return bin;
}

double binToYawDeg(int bin)
{
  return normalizeAngleDeg(static_cast<double>(bin) * TURN_DEG);
}

LatticeKey quantize(double x, double y, int yaw_bin, double step_m)
{
  LatticeKey k;
  k.xi = static_cast<int>(std::lround(x / step_m));
  k.yi = static_cast<int>(std::lround(y / step_m));
  k.yaw = ((yaw_bin % YAW_BINS) + YAW_BINS) % YAW_BINS;
  return k;
}

bool worldToCell(
  const nav_msgs::msg::OccupancyGrid & grid,
  double x_m, double y_m, int * col, int * row)
{
  if (grid.info.width == 0 || grid.info.height == 0 || grid.info.resolution <= 0.0) {
    return false;
  }
  const double res = grid.info.resolution;
  const int c = static_cast<int>(std::floor(
      (x_m - grid.info.origin.position.x) / res));
  const int r = static_cast<int>(std::floor(
      (y_m - grid.info.origin.position.y) / res));
  if (c < 0 || r < 0 ||
    c >= static_cast<int>(grid.info.width) ||
    r >= static_cast<int>(grid.info.height))
  {
    return false;
  }
  *col = c;
  *row = r;
  return true;
}

int8_t cellValue(const nav_msgs::msg::OccupancyGrid & grid, int col, int row)
{
  const std::size_t idx =
    static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(col);
  if (idx >= grid.data.size()) {
    return 100;
  }
  return grid.data[idx];
}

bool poseClear(
  const nav_msgs::msg::OccupancyGrid & grid,
  double x, double y,
  const DiscreteNavConfig & cfg)
{
  const double res = grid.info.resolution;
  if (res <= 0.0) {
    return false;
  }
  const int inflate = std::max(
    0, static_cast<int>(std::ceil(cfg.robot_radius_m / res)));
  int col = 0;
  int row = 0;
  if (!worldToCell(grid, x, y, &col, &row)) {
    return false;
  }
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  for (int dr = -inflate; dr <= inflate; ++dr) {
    for (int dc = -inflate; dc <= inflate; ++dc) {
      if (dc * dc + dr * dr > inflate * inflate) {
        continue;
      }
      const int c = col + dc;
      const int r = row + dr;
      if (c < 0 || r < 0 || c >= w || r >= h) {
        return false;
      }
      const int8_t v = cellValue(grid, c, r);
      if (v < 0) {
        if (!cfg.allow_unknown) {
          return false;
        }
        continue;
      }
      if (v >= cfg.occupied_threshold) {
        return false;
      }
    }
  }
  return true;
}

void appendTurnSteps(std::vector<NavigationStep> & plan, double turn_deg)
{
  if (std::abs(turn_deg) < 1e-6) {
    return;
  }
  const uint32_t steps = static_cast<uint32_t>(std::lround(std::abs(turn_deg) / TURN_DEG));
  if (steps == 0) {
    return;
  }
  NavigationStep step;
  step.direction = (turn_deg > 0.0) ? DIR_TURN_LEFT : DIR_TURN_RIGHT;
  step.steps = steps;
  plan.push_back(step);
}

}  // namespace

double normalizeAngleDeg(double angle_deg)
{
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg < -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

double shortestTurnDeg(double current_yaw_deg, double target_yaw_deg)
{
  return normalizeAngleDeg(target_yaw_deg - current_yaw_deg);
}

std::vector<NavigationStep> compactDiscreteActions(
  const std::vector<uint8_t> & unit_actions)
{
  std::vector<NavigationStep> plan;
  for (uint8_t a : unit_actions) {
    if (plan.empty() || plan.back().direction != a) {
      plan.push_back(NavigationStep{a, 1});
    } else {
      plan.back().steps += 1;
    }
  }
  return plan;
}

std::vector<NavigationStep> planToPose(
  double cx, double cy, double cyaw_deg,
  double gx, double gy, double gyaw_deg)
{
  std::vector<NavigationStep> plan;

  const double dx = gx - cx;
  const double dy = gy - cy;
  const double distance = std::hypot(dx, dy);

  if (distance > 1e-6) {
    const double bearing_deg = normalizeAngleDeg(std::atan2(dy, dx) * 180.0 / kPi);
    appendTurnSteps(plan, shortestTurnDeg(cyaw_deg, bearing_deg));

    const uint32_t forward_steps = static_cast<uint32_t>(std::lround(distance / STEP_M));
    if (forward_steps > 0) {
      plan.push_back({DIR_FORWARD, forward_steps});
    }

    appendTurnSteps(plan, shortestTurnDeg(bearing_deg, gyaw_deg));
  } else {
    appendTurnSteps(plan, shortestTurnDeg(cyaw_deg, gyaw_deg));
  }

  return plan;
}

DiscretePlanResult planOnOccupancy(
  const nav_msgs::msg::OccupancyGrid & grid,
  double cx, double cy, double cyaw_deg,
  double gx, double gy, double /*gyaw_deg*/,
  const DiscreteNavConfig & cfg)
{
  DiscretePlanResult out;
  if (grid.info.width == 0 || grid.info.height == 0 ||
    grid.info.resolution <= 0.0 || grid.data.empty())
  {
    return out;
  }

  if (std::hypot(gx - cx, gy - cy) <= cfg.goal_tol_m) {
    out.ok = true;
    return out;
  }

  // Allow planning when start is slightly inflated (wedged); still reject hard occ.
  DiscreteNavConfig start_cfg = cfg;
  start_cfg.robot_radius_m = std::min(cfg.robot_radius_m, 0.05);
  if (!poseClear(grid, cx, cy, start_cfg)) {
    return out;
  }

  const double step = cfg.step_m > 0.0 ? cfg.step_m : STEP_M;
  std::vector<SearchNode> nodes;
  nodes.reserve(8192);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenCmp> open;
  std::unordered_map<LatticeKey, double, LatticeKeyHash> best_g;

  SearchNode start;
  start.x = cx;
  start.y = cy;
  start.key = quantize(cx, cy, yawToBin(cyaw_deg), step);
  start.g = 0.0;
  start.parent = -1;
  start.action = 255;
  nodes.push_back(start);
  best_g[start.key] = 0.0;
  open.push(OpenEntry{0, std::hypot(gx - cx, gy - cy)});

  int goal_idx = -1;

  while (!open.empty() && out.expansions < cfg.max_expansions) {
    const OpenEntry ent = open.top();
    open.pop();
    ++out.expansions;
    const int cur_idx = ent.idx;
    const SearchNode cur = nodes[static_cast<std::size_t>(cur_idx)];

    auto bit = best_g.find(cur.key);
    if (bit != best_g.end() && cur.g > bit->second + 1e-9) {
      continue;
    }

    if (std::hypot(gx - cur.x, gy - cur.y) <= cfg.goal_tol_m) {
      goal_idx = cur_idx;
      break;
    }

    const double yaw_rad = binToYawDeg(cur.key.yaw) * kPi / 180.0;
    const double cos_y = std::cos(yaw_rad);
    const double sin_y = std::sin(yaw_rad);

    struct Candidate
    {
      uint8_t action;
      double nx;
      double ny;
      int nyaw;
      double cost;
    };
    const Candidate cands[4] = {
      {DIR_FORWARD, cur.x + step * cos_y, cur.y + step * sin_y, cur.key.yaw, 1.0},
      {DIR_BACKWARD, cur.x - step * cos_y, cur.y - step * sin_y, cur.key.yaw, 1.25},
      {DIR_TURN_LEFT, cur.x, cur.y, (cur.key.yaw + 1) % YAW_BINS, 0.55},
      {DIR_TURN_RIGHT, cur.x, cur.y, (cur.key.yaw + YAW_BINS - 1) % YAW_BINS, 0.55},
    };

    for (const Candidate & cand : cands) {
      if ((cand.action == DIR_FORWARD || cand.action == DIR_BACKWARD) &&
        !poseClear(grid, cand.nx, cand.ny, cfg))
      {
        continue;
      }
      SearchNode nxt;
      nxt.x = cand.nx;
      nxt.y = cand.ny;
      nxt.key = quantize(cand.nx, cand.ny, cand.nyaw, step);
      nxt.g = cur.g + cand.cost;
      nxt.parent = cur_idx;
      nxt.action = cand.action;

      auto existing = best_g.find(nxt.key);
      if (existing != best_g.end() && nxt.g >= existing->second - 1e-9) {
        continue;
      }
      best_g[nxt.key] = nxt.g;
      const int nxt_idx = static_cast<int>(nodes.size());
      nodes.push_back(nxt);
      open.push(OpenEntry{nxt_idx, nxt.g + std::hypot(gx - nxt.x, gy - nxt.y)});
    }
  }

  if (goal_idx < 0) {
    return out;
  }

  std::vector<uint8_t> actions;
  for (int idx = goal_idx; idx >= 0; ) {
    const SearchNode & n = nodes[static_cast<std::size_t>(idx)];
    if (n.parent < 0) {
      break;
    }
    actions.push_back(n.action);
    idx = n.parent;
  }
  std::reverse(actions.begin(), actions.end());
  out.steps = compactDiscreteActions(actions);
  out.ok = true;
  return out;
}

bool discretePathExists(
  const nav_msgs::msg::OccupancyGrid & grid,
  double cx, double cy, double cyaw_deg,
  double gx, double gy,
  const DiscreteNavConfig & cfg)
{
  return planOnOccupancy(grid, cx, cy, cyaw_deg, gx, gy, 0.0, cfg).ok;
}

}  // namespace explorer_mission
