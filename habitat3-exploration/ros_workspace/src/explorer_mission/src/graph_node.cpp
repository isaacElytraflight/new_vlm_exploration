#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <explorer_msgs/msg/edge.hpp>
#include <explorer_msgs/msg/graph.hpp>
#include <explorer_msgs/msg/vertex.hpp>
#include <explorer_msgs/srv/add_edge.hpp>
#include <explorer_msgs/srv/run_dijkstra.hpp>

#include "explorer_mission/graph_logic.hpp"

class GraphNode : public rclcpp::Node
{
public:
  GraphNode()
  : Node("graph_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    pub_graph_ = create_publisher<explorer_msgs::msg::Graph>("graph_node/graph", rclcpp::QoS(1).transient_local());
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "graph_node/graph_markers", rclcpp::QoS(1).transient_local());
    pub_path_ = create_publisher<nav_msgs::msg::Path>("graph_node/backtrack_path", 10);

    srv_add_edge_ = create_service<explorer_msgs::srv::AddEdge>(
      "graph_node/graph/add_edge",
      std::bind(&GraphNode::addEdgeSrv, this, std::placeholders::_1, std::placeholders::_2));
    srv_run_dij_ = create_service<explorer_msgs::srv::RunDijkstra>(
      "graph_node/graph/run_dijkstra",
      std::bind(&GraphNode::runDijkstraSrv, this, std::placeholders::_1, std::placeholders::_2));
    srv_reset_ = create_service<std_srvs::srv::Empty>(
      "graph_node/graph/reset",
      std::bind(&GraphNode::resetSrv, this, std::placeholders::_1, std::placeholders::_2));

    publishGraph();
  }

private:
  void publishGraph()
  {
    explorer_msgs::msg::Graph gg;
    gg.next_id = graph_.nextId();
    gg.header.frame_id = frame_id_;
    gg.header.stamp = now();

    for (const auto & v : graph_.vertices()) {
      explorer_msgs::msg::Vertex vertex;
      vertex.id = v.id;
      vertex.x = v.x;
      vertex.y = v.y;
      gg.vertices.push_back(vertex);
    }

    for (const auto & e : graph_.edges()) {
      explorer_msgs::msg::Edge edge;
      edge.u_id = e.u_id;
      edge.v_id = e.v_id;
      edge.weight = e.weight;
      gg.edges.push_back(edge);
    }

    pub_graph_->publish(gg);
    pub_markers_->publish(makeMarkers(gg.header.stamp));
  }

  visualization_msgs::msg::MarkerArray makeMarkers(const builtin_interfaces::msg::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker verts;
    visualization_msgs::msg::Marker lines;

    verts.header.frame_id = lines.header.frame_id = frame_id_;
    verts.header.stamp = lines.header.stamp = stamp;
    verts.ns = lines.ns = "graph";
    verts.id = 0;
    lines.id = 1;
    verts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    verts.action = lines.action = visualization_msgs::msg::Marker::ADD;
    verts.scale.x = verts.scale.y = verts.scale.z = 0.08;
    lines.scale.x = 0.03;
    verts.color.r = 0.2f;
    verts.color.g = 0.8f;
    verts.color.b = 0.2f;
    lines.color.r = 0.8f;
    lines.color.g = 0.8f;
    lines.color.b = 0.8f;
    verts.color.a = lines.color.a = 1.0f;

    const auto vertices = graph_.vertices();
    const auto edges = graph_.edges();

    for (const auto & v : vertices) {
      geometry_msgs::msg::Point p;
      p.x = v.x;
      p.y = v.y;
      p.z = 0.0;
      verts.points.push_back(p);
    }

    std::unordered_map<uint32_t, explorer_mission::GraphVertex> id_map;
    for (const auto & v : vertices) {
      id_map[v.id] = v;
    }

    for (const auto & e : edges) {
      geometry_msgs::msg::Point pa;
      geometry_msgs::msg::Point pb;
      pa.x = id_map[e.u_id].x;
      pa.y = id_map[e.u_id].y;
      pa.z = 0.0;
      pb.x = id_map[e.v_id].x;
      pb.y = id_map[e.v_id].y;
      pb.z = 0.0;
      lines.points.push_back(pa);
      lines.points.push_back(pb);
    }

    arr.markers.push_back(lines);
    arr.markers.push_back(verts);
    return arr;
  }

  void fillGraphMsg(explorer_msgs::msg::Graph & gg)
  {
    gg.next_id = graph_.nextId();
    gg.header.frame_id = frame_id_;
    gg.header.stamp = now();
    for (const auto & v : graph_.vertices()) {
      explorer_msgs::msg::Vertex vertex;
      vertex.id = v.id;
      vertex.x = v.x;
      vertex.y = v.y;
      gg.vertices.push_back(vertex);
    }
    for (const auto & e : graph_.edges()) {
      explorer_msgs::msg::Edge edge;
      edge.u_id = e.u_id;
      edge.v_id = e.v_id;
      edge.weight = e.weight;
      gg.edges.push_back(edge);
    }
  }

  void addEdgeSrv(
    const std::shared_ptr<explorer_msgs::srv::AddEdge::Request> req,
    std::shared_ptr<explorer_msgs::srv::AddEdge::Response> res)
  {
    const auto result = graph_.addEdge(
      req->a.id, req->a.x, req->a.y,
      req->b.id, req->b.x, req->b.y,
      req->weight_override, req->snap_tolerance_m);

    res->a_out.id = result.a_out.id;
    res->a_out.x = result.a_out.x;
    res->a_out.y = result.a_out.y;
    res->b_out.id = result.b_out.id;
    res->b_out.x = result.b_out.x;
    res->b_out.y = result.b_out.y;
    res->created_a = result.created_a;
    res->created_b = result.created_b;
    res->created_edge = result.created_edge;
    fillGraphMsg(res->graph);
    publishGraph();
  }

  void runDijkstraSrv(
    const std::shared_ptr<explorer_msgs::srv::RunDijkstra::Request> req,
    std::shared_ptr<explorer_msgs::srv::RunDijkstra::Response> res)
  {
    const auto result = graph_.runDijkstra(
      req->start.x, req->start.y,
      req->goal.x, req->goal.y,
      req->snap_tolerance_m, frame_id_);

    res->success = result.success;
    res->message = result.message;
    res->path = result.path;
    if (result.success) {
      res->path.header.stamp = now();
      pub_path_->publish(res->path);
    }
  }

  void resetSrv(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
  {
    graph_.reset();
    publishGraph();
  }

  std::string frame_id_;
  explorer_mission::GraphLogic graph_;

  rclcpp::Publisher<explorer_msgs::msg::Graph>::SharedPtr pub_graph_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Service<explorer_msgs::srv::AddEdge>::SharedPtr srv_add_edge_;
  rclcpp::Service<explorer_msgs::srv::RunDijkstra>::SharedPtr srv_run_dij_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphNode>());
  rclcpp::shutdown();
  return 0;
}
