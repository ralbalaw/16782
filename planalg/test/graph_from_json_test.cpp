#include "planalg/graph/graph.hpp"

#include <cassert>

int main()
{
  const auto graph = planalg::graph::Graph::fromJson(planalg::graph::Map::kRomanian);
  assert(graph.getVertexCount() == 20);

  return 0;
}
