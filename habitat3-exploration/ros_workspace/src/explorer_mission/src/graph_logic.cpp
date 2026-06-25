#include "explorer_mission/graph_logic.hpp"

#include <algorithm>
#include <cmath>

#include <boost/range/iterator_range.hpp>

namespace explorer_mission
{

GraphLogic::GraphLogic() = default;

void GraphLogic::reset()
{
  std::lock_guard<std::mutex> lk(mu_);
  g_.clear();
  id2v_.clear();
  key2id_.clear();
  next_id_ = 1;
  edges_list_.clear();
}

std::vector<GraphVertex> GraphLogic::vertices() const
{
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<GraphVertex> out;
  out.reserve(id2v_.size());
  for (const auto & kv : id2v_) {
    out.push_back({kv.first, g_[kv.second].x, g_[kv.second].y});
  }
  return out;
}

std::vector<GraphEdge> GraphLogic::edges() const
{
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<GraphEdge> out;
  out.reserve(edges_list_.size());
  for (const auto & e : edges_list_) {
    out.push_back({std::get<0>(e), std::get<1>(e), std::get<2>(e)});
  }
  return out;
}

uint32_t GraphLogic::getOrCreate(double x, double y, double tol, bool & created)
{
  const VertexKey key{x, y};

  const auto it = key2id_.find(key);
  if (it != key2id_.end()) {
    created = false;
    return it->second;
  }

  for (const auto & kv : key2id_) {
    if (std::hypot(kv.first.x - x, kv.first.y - y) <= tol) {
      created = false;
      return kv.second;
    }
  }

  const VDesc v = boost::add_vertex(g_);
  g_[v] = {x, y};
  const uint32_t id = next_id_++;
  id2v_[id] = v;
  key2id_[{x, y}] = id;
  created = true;
  return id;
}

bool GraphLogic::addEdgeInternal(uint32_t a_id, uint32_t b_id, double w, bool & created_edge)
{
  const VDesc va = id2v_.at(a_id);
  const VDesc vb = id2v_.at(b_id);

  created_edge = true;
  for (const auto ep : boost::make_iterator_range(boost::out_edges(va, g_))) {
    if (boost::target(ep, g_) == vb) {
      created_edge = false;
      break;
    }
  }

  if (created_edge) {
    boost::graph_traits<GraphT>::edge_descriptor e;
    bool inserted = false;
    boost::tie(e, inserted) = boost::add_edge(va, vb, g_);
    if (inserted) {
      boost::put(boost::edge_weight, g_, e, w);
      edges_list_.push_back({a_id, b_id, w});
    } else {
      created_edge = false;
    }
  }
  return true;
}

AddEdgeResult GraphLogic::addEdge(
  uint32_t a_id, double ax, double ay,
  uint32_t b_id, double bx, double by,
  double weight_override,
  double snap_tolerance_m)
{
  std::lock_guard<std::mutex> lk(mu_);

  AddEdgeResult res;
  bool ca = false;
  bool cb = false;

  const uint32_t resolved_a = (a_id > 0) ? a_id : getOrCreate(ax, ay, snap_tolerance_m, ca);
  const uint32_t resolved_b = (b_id > 0) ? b_id : getOrCreate(bx, by, snap_tolerance_m, cb);

  const double w = (weight_override > 0.0) ?
    weight_override :
    std::hypot(
      g_[id2v_[resolved_a]].x - g_[id2v_[resolved_b]].x,
      g_[id2v_[resolved_a]].y - g_[id2v_[resolved_b]].y);

  bool created_edge = false;
  addEdgeInternal(resolved_a, resolved_b, w, created_edge);

  res.a_out = {resolved_a, g_[id2v_[resolved_a]].x, g_[id2v_[resolved_a]].y};
  res.b_out = {resolved_b, g_[id2v_[resolved_b]].x, g_[id2v_[resolved_b]].y};
  res.created_a = ca;
  res.created_b = cb;
  res.created_edge = created_edge;
  return res;
}

DijkstraResult GraphLogic::runDijkstra(
  double start_x, double start_y,
  double goal_x, double goal_y,
  double snap_tolerance_m,
  const std::string & frame_id) const
{
  std::lock_guard<std::mutex> lk(mu_);

  DijkstraResult res;

  auto snap = [&](double x, double y) -> int {
      int best = -1;
      double best_d = snap_tolerance_m;
      for (const auto & kv : id2v_) {
        const double d = std::hypot(g_[kv.second].x - x, g_[kv.second].y - y);
        if (d <= best_d) {
          best_d = d;
          best = static_cast<int>(kv.first);
        }
      }
      return best;
    };

  const int s_id = snap(start_x, start_y);
  const int g_id = snap(goal_x, goal_y);
  if (s_id < 0 || g_id < 0) {
    res.success = false;
    res.message = "Start or goal not within tolerance of a vertex.";
    return res;
  }

  const VDesc s = id2v_.at(static_cast<uint32_t>(s_id));
  const VDesc t = id2v_.at(static_cast<uint32_t>(g_id));

  const size_t n = boost::num_vertices(g_);
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<VDesc> pred(n, GraphT::null_vertex());

  const auto indexmap = get(boost::vertex_index, g_);
  dist[indexmap[s]] = 0.0;

  boost::dijkstra_shortest_paths(
    g_, s,
    boost::weight_map(get(boost::edge_weight, g_))
    .distance_map(dist.data())
    .predecessor_map(pred.data()));

  if (pred[indexmap[t]] == GraphT::null_vertex() && s != t) {
    res.success = false;
    res.message = "No path found.";
    return res;
  }

  std::vector<VDesc> seq;
  for (VDesc v = t; v != GraphT::null_vertex(); v = pred[indexmap[v]]) {
    seq.push_back(v);
    if (v == s) {
      break;
    }
  }
  std::reverse(seq.begin(), seq.end());

  res.path.header.frame_id = frame_id;
  for (const auto v : seq) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = res.path.header;
    ps.pose.position.x = g_[v].x;
    ps.pose.position.y = g_[v].y;
    ps.pose.orientation.w = 1.0;
    res.path.poses.push_back(ps);
  }

  res.success = true;
  res.message = "ok";
  return res;
}

}  // namespace explorer_mission
