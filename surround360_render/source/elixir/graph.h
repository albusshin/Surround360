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

    static size_t totalNodes;
    static size_t numBatches;
    const size_t batchSize = 1;

    Graph (size_t totalNodes, size_t numFrames) {
      Graph::totalNodes = totalNodes;
      Graph::numBatches = (numFrames + batchSize - 1) / batchSize;
      pthread_mutex_init(&graphlock);
    }

    vector<Node *> getRunnableJobs();

    void updateGraphNode(int nodeKey);

    void assertThatInvariantsHold();

    // All the nodes in one layer, with probably different batchIds
    // Mapping NodeKey -> nodes
    unordered_map<int, Node *> nodes;

  private:
    // TODO:
    // Should be a variable that can be adjust based on core number and first
    // layer node number. Should > (core number / first layer node number)
    // const int layers_threshold = 3;

    pthread_mutex_t graphlock;

    Node *getRunnableJob();

    void lock();

    void unlock();

  };

  class Node {
  public:
    int nodeId;
    int batchId;
    int depth;

    // The graph this node belongs to
    Graph *graph;

    Kernel *kernel;

    // parents, order is the same with Data input order
    vector<int> parents;

    vector<int> children;

    Node(int nodeId,
         int batchId,
         int depth,
         Graph *graph,
         vector<int> &parents,
         vector<int> &children)
      : nodeId(nodeId),
        batchId(batchId),
        graph(graph),
        parents(parents),
        children(children),
        depth(depth){}

    ~Node() {
        delete kernel;
    }

    static int getNodeKeyByIds(int nodeId, int batchId);

    static int getNodeIdByNodeKey(int nodeKey);

    static int getBatchIdByNodeKey(int nodeKey);
  };

}

#endif /* ELIXIR_GRAPH_H */
