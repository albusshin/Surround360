#ifndef ELIXIR_GRAPH_H
#define ELIXIR_GRAPH_H

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "pthread.h"
#include "kernel.h"

namespace elixir {
  using namespace std;

  class Node;

  class Graph {

  public:

    static int totalNodes;

    Graph (int totalNodes, int numBatches)
      : numBatches(numBatches) {
      Graph::totalNodes = totalNodes;
    }

    vector<Node *> getRunnableJobs();

    // All the nodes in one layer, with probably different batchIds
    vector<Node *> nodes;

    int numBatches;

  private:
    pthread_mutex_t graphlock;

    Node *getRunnableJob();

    void lock();

    void unlock();

  };

  class Node {
  public:
    int nodeId;
    int batchId;

    // The graph this node belongs to
    Graph *graph;

    Kernel *kernel;

    // parents, order is the same with Data input order
    vector<int> parents;

    unordered_set<int> children;

    Node(int nodeId,
         int batchId,
         Graph *graph,
         vector<int> &parents,
         unordered_set<int> &children)
      : nodeId(nodeId),
        batchId(batchId),
        graph(graph),
        parents(parents),
        children(children) {}

    static int getNodeKeyByIds(int nodeId, int batchId);

    static int getNodeIdByNodeKey(int nodeKey);

    static int getBatchIdByNodeKey(int nodeKey);
  };

}

#endif /* ELIXIR_GRAPH_H */
