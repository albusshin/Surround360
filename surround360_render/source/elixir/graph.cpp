#include <iostream>
#include <vector>
#include <pthread.h>
#include <unordered_map>
#include <unordered_set>
#include <assert.h>

#include "graph.h"
#include "scheduler.h"
#include "nullbuf.h"
#include "worker.h"

#define DEBUG

//#define logger cout
#define logger null_stream

namespace elixir {
  using namespace std;

  size_t Graph::totalNodes;
  size_t Graph::numBatches;

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

      logger << "[Graph T"
             << Worker::getWorkerId()
             << "]\t"
             << "getRunnableJob(): nodeKey: "
             << nodeKey
             << " ids: "
             << node->nodeId
             << ", "
             << node->batchId
             << endl;
      assert(node != nullptr);

      if (node->batchId >= numBatches) {
        continue;
      }

      // Criterion: check if the batchId of the job is too deep.
      // If so, we don't add this job to the queue even if it's runnable.

      bool tooDeep = Scheduler::getScheduler().isJobBatchTooDeep(node->batchId);

      if (tooDeep) {
        continue;
      }
      logger << "[Graph T"
             << Worker::getWorkerId()
             << "]\t"
             << "getRunnableJob(): not too deep: "
             << nodeKey
             << " ids: "
             << node->nodeId
             << ", "
             << node->batchId
             << endl;

      bool nodeIsRunnable = true;
      // Check if all the parents are finished
      for (int parentNodeKey : node->parents) {
        if (!Scheduler::getScheduler().isJobFinished(parentNodeKey)) {
          nodeIsRunnable = false;
          break;
        }
      }
      if (nodeIsRunnable) {
        logger << "[Graph T"
               << Worker::getWorkerId()
               << "]\t"
               << ": getRunnableJob(): node is runnable:"
               << nodeKey
               << " ids: "
               << node->nodeId
               << ", "
               << node->batchId
               << endl;
        // Return runnable job
        Node *result = new Node(node->nodeId, node->batchId, node->depth,
                                node->graph, node->parents, node->children);
        result->kernel = node->kernel->clone();

        updateGraphNode(nodeKey);
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

  void Graph::updateGraphNode(int nodeKey) {
    //TODO check implementation correctness
    assertThatInvariantsHold();

    Node *node = nodes[nodeKey];
    assert(node != nullptr);
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
    node->kernel->updateToNextLayer();

    nodes.erase(nodeKey);
    nodes[newKey] = node;
    logger << "[Graph T"
           << Worker::getWorkerId()
           << "]\t"
           << " after erasing "
           << nodeKey
           << " from graph.nodes, graph.nodes.size() == "
           << nodes.size()
           << ", newKey == "
           << newKey
           << endl;

    assertThatInvariantsHold();
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
    logger << "[Graph T"
           << Worker::getWorkerId()
           << "]\t"
           << ": lock()"
           << endl;
    pthread_mutex_lock(&graphlock);
    logger << "[Graph T"
           << Worker::getWorkerId()
           << "]\t"
           << ": acquired lock."
           << endl;
  }

  void Graph::unlock() {
    logger << "[Graph T"
           << Worker::getWorkerId()
           << "]\t"
           << ": unlock()"
           << endl;
    pthread_mutex_unlock(&graphlock);
  }

  //publics for Node
  int Node::getNodeKeyByIds(int nodeId, int batchId) {
    return int(batchId * Graph::totalNodes + nodeId);
  }

  int Node::getNodeIdByNodeKey(int nodeKey) {
    return int(nodeKey % Graph::totalNodes);
  }

  int Node::getBatchIdByNodeKey(int nodeKey) {
    return int(nodeKey / Graph::totalNodes);
  }

};
