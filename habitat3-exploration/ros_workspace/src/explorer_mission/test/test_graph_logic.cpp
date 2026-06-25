#include <gtest/gtest.h>

#include "explorer_mission/graph_logic.hpp"

TEST(GraphLogic, AddEdgeCreatesVertices)
{
  explorer_mission::GraphLogic graph;
  const auto result = graph.addEdge(0, 0.0, 0.0, 0, 1.0, 0.0, 0.0, 0.1);
  EXPECT_GT(result.a_out.id, 0u);
  EXPECT_GT(result.b_out.id, 0u);
  EXPECT_EQ(graph.vertices().size(), 2u);
}

TEST(GraphLogic, DijkstraFindsPath)
{
  explorer_mission::GraphLogic graph;
  auto r1 = graph.addEdge(0, 0.0, 0.0, 0, 1.0, 0.0, 0.0, 0.1);
  auto r2 = graph.addEdge(r1.b_out.id, r1.b_out.x, r1.b_out.y, 0, 2.0, 0.0, 0.0, 0.1);

  const auto path = graph.runDijkstra(0.0, 0.0, 2.0, 0.0, 0.5, "map");
  EXPECT_TRUE(path.success);
  EXPECT_GE(path.path.poses.size(), 2u);
}

TEST(GraphLogic, ResetClearsGraph)
{
  explorer_mission::GraphLogic graph;
  graph.addEdge(0, 0.0, 0.0, 0, 1.0, 0.0, 0.0, 0.1);
  graph.reset();
  EXPECT_EQ(graph.vertices().size(), 0u);
  EXPECT_EQ(graph.nextId(), 1u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
