#ifndef ELIXIR_GRAPH_H
#define ELIXIR_GRAPH_H

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "kernel.h"

namespace elixir {
  using namespace std;

  class Node;

  class Graph {

    int totalNodes;

  public:

    Graph (int totalNodes);

    vector<Node *> getRunnableJobs();

  private:
    // All the nodes in one layer, with probably different batchIds
    vector<Node *> nodes;

  };

  class Node {
    int nodeId;
    int batchId;

    // The graph this node belongs to
    Graph *graph;

    Kernel *kernel;

    // parents, order is the same with Data input order
    vector<int> parents;

    unordered_set<int> children;

  public:

    static int getNodeKeyByIds(int nodeId, int batchId);

    static int getNodeIdByNodeKey(int nodeKey);
  };

}

#endif /* ELIXIR_GRAPH_H */
