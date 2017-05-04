#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "graph.h"
#include "scheduler.h"

namespace elixir {
  using namespace std;

  //publics for Graph
  vector<Node *> Graph::getRunnableJobs() {
    vector<Node *> result;
    Node *node;
    while ((node = getRunnableJob()) != nullptr) {
      result.push_back(node);
    }
    return result;
  }

  Node *Graph::getRunnableJob() {
    this->lock();

    for (Node *node: nodes) {
      bool nodeIsRunnable = true;
      // Check if all the parents are finisehd
      for (int parentNodeKey : node->parents) {
        if (!Scheduler::getScheduler().isJobFinished(parentNodeKey)) {
          nodeIsRunnable = false;
          break;
        }
      }
      // Return runnable job
      if (nodeIsRunnable) {

        this->unlock();
        return node;
      }
    }

    this->unlock();
    return nullptr;
  }

  void Graph::lock() {
    pthread_mutex_lock(&graphlock);
  }

  void Graph::unlock() {
    pthread_mutex_unlock(&graphlock);
  }

  //publics for Node
  int Node::getNodeKeyByIds(int nodeId, int batchId) {
    return batchId * Graph::totalNodes + nodeId;
  }

  int Node::getNodeIdByNodeKey(int nodeKey) {
    return nodeKey % Graph::totalNodes;
  }

  int Node::getBatchIdByNodeKey(int nodeKey) {
    return nodeKey / Graph::totalNodes;
  }

};
