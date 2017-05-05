#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <assert.h>

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
    assertThatInvariantsHold();

    for (auto pair: nodes) {
      int nodeKey = pair.first;
      Node *node = pair.second;
      assert(node != nullptr);

      // Criterion: check if the batchId of the job is too deep.
      // If so, we don't add this job to the queue even if it's runnable.
      int minBatchIdInRunnableQueue =
        Scheduler::getScheduler().getMinBatchIdInRunnableJobs();

      if (minBatchIdInRunnableQueue + layers_threshold < node->batchId) {
        continue;
      }
      
      bool nodeIsRunnable = true;
      // Check if all the parents are finished
      for (int parentNodeKey : node->parents) {
        if (!Scheduler::getScheduler().isJobFinished(parentNodeKey)) {
          nodeIsRunnable = false;
          break;
        }
      }
      if (nodeIsRunnable) {
        // Return runnable job
        Node *result = new Node(node->nodeId, node->batchId, node->graph,
                                node->parents, node->children);
        UpdateGraphNode(nodeKey);
        assert(result != nullptr);

        assertThatInvariantsHold();
        this->unlock();
        return result;
      }
    }

    assertThatInvariantsHold();
    this->unlock();
    return nullptr;
  }

  //TODO
  void Graph::UpdateGraphNode(int nodeKey) {
    //TODO check implementation correctness
    lock();
    assertThatInvariantsHold();

    Node *node = nodes[nodeKey];
    for (size_t i = 0; i < node->parents.size(); ++i) {
      // Update every dependency to the next layer.
      node->parents[i] += totalNodes;
    }
    for (size_t i = 0; i < node->children.size(); ++i) {
      // Update every dependency to the next layer.
      node->children[i] += totalNodes;
    }

    // remove from nodes map and reinsert with new nodeKey.
    int newKey = nodeKey + totalNodes;
    // update node batchId
    node->batchId++;
    nodes.erase(nodeKey);
    nodes[newKey] = node;

    assertThatInvariantsHold();
    unlock();
  }

  void Graph::assertThatInvariantsHold() {
#ifdef DEBUG
    assert(nodes.size() == totalNodes);
    for (auto pair : nodes) {
      int nodeKey = pair.first;
      Node *node = pair.second;
      assert(Node::getNodeKeyByIds(node->nodeId, node->batchId) == nodeKey);
      assert(Node::getNodeIdByNodeKey(nodeKey) == node->nodeId);
      assert(Node::getBatchIdByNodeKey(nodeKey) == node->batchId);
      assert(node->batchId >= -1 && node->batchId <= numBatches);
      assert(node->kernel != nullptr);
      assert(node->graph == Scheduler::getScheduler().graph);
      assert(!(node->parents.empty() && node->children.empty()));
    }
#endif /* DEBUG */
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
