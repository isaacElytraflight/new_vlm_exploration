#pragma once

#include <cstdint>
#include <limits>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace explorer_mission
{

struct VertexKey
{
  double x{0.0};
  double y{0.0};
};

struct KeyHash
{
  size_t operator()(const VertexKey & k) const noexcept
  {
    const double q = 0.01;
    const int64_t ix = llround(k.x / q);
    const int64_t iy = llround(k.y / q);
    const uint64_t mix = (static_cast<uint64_t>(ix) << 32) ^ static_cast<uint64_t>(iy);
    return std::hash<uint64_t>()(mix);
  }
};

struct KeyEq
{
  bool operator()(const VertexKey & a, const VertexKey & b) const noexcept
  {
    return std::fabs(a.x - b.x) < 1e-3 && std::fabs(a.y - b.y) < 1e-3;
  }
};

struct GraphVertex
{
  uint32_t id{0};
  double x{0.0};
  double y{0.0};
};

struct GraphEdge
{
  uint32_t u_id{0};
  uint32_t v_id{0};
  double weight{0.0};
};

struct AddEdgeResult
{
  GraphVertex a_out;
  GraphVertex b_out;
  bool created_a{false};
  bool created_b{false};
  bool created_edge{false};
};

struct DijkstraResult
{
  bool success{false};
  std::string message;
  nav_msgs::msg::Path path;
};

/// Boost.Graph-backed exploration graph (extracted from graph_node.cpp).
class GraphLogic
{
public:
  using GraphT = boost::adjacency_list<
    boost::listS,
    boost::vecS,
    boost::undirectedS,
    VertexKey,
    boost::property<boost::edge_weight_t, double>>;

  using VDesc = GraphT::vertex_descriptor;

  GraphLogic();

  void reset();

  uint32_t nextId() const {return next_id_;}

  std::vector<GraphVertex> vertices() const;

  std::vector<GraphEdge> edges() const;

  AddEdgeResult addEdge(
    uint32_t a_id, double ax, double ay,
    uint32_t b_id, double bx, double by,
    double weight_override,
    double snap_tolerance_m);

  DijkstraResult runDijkstra(
    double start_x, double start_y,
    double goal_x, double goal_y,
    double snap_tolerance_m,
    const std::string & frame_id) const;

private:
  uint32_t getOrCreate(double x, double y, double tol, bool & created);

  bool addEdgeInternal(uint32_t a_id, uint32_t b_id, double w, bool & created_edge);

  mutable std::mutex mu_;
  GraphT g_;
  std::unordered_map<uint32_t, VDesc> id2v_;
  std::unordered_map<VertexKey, uint32_t, KeyHash, KeyEq> key2id_;
  uint32_t next_id_{1};
  std::vector<std::tuple<uint32_t, uint32_t, double>> edges_list_;
};

}  // namespace explorer_mission
