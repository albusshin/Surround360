#include <pthread.h>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include <assert.h>

#include "scheduler.h"
#include "graph.h"

namespace elixir {

  using namespace std;

  void Scheduler::init(Graph *graph) {
    this->graph = graph;

    // parse graph to find every runnable job
    vector<Node *> runnableJobs = this->graph->getRunnableJobs();

    for (vector<Node *>::iterator it = runnableJobs.begin();
         it != runnableJobs.end(); ++it) {
      this->runnableJobs.insert(*it);
    }
  }

  Node *Scheduler::scheduleJob(int workerId) {
    //TODO implement
    // select job from job queue
  }

  bool Scheduler::allFinished() {
    lock();
    assertThatInvariantsHold();
    if (!runnableJobs.empty()) {
      unlock();
      return false;
    } else if (!runningJobs.empty()) {
      unlock();
      return false;
    }
    else {
      for (auto pair: graph->nodes) {
        Node *node = pair.second;
        if (node->batchId < graph->numBatches) {
          unlock();
          return false;
        }
      }
    }
    assertThatInvariantsHold();
    assert(finished.size() == graph->totalNodes * graph->numBatches);
    unlock();
    return true;
  }

  void Scheduler::onJobFinishing(int nodeKey, Data *outputData, int workerId) {
    lock();
    assertThatInvariantsHold();
    // update state
    // update data
    dataMap[nodeKey] = outputData;

    // cleanup
    Node *graphNode = graph->nodes[nodeKey];
    Node *finishingNode = runningJobs[workerId];

    assert(graphNode != finishingNode);

    // remove from runningJobs
    runningJobs.erase(nodeKey);

    // set job finished
    markJobFinished(nodeKey);

    //Free up memory
    delete finishingNode;

    dataMapCleanup();

    assertThatInvariantsHold();
    unlock();
  }

  void Scheduler::dataMapCleanup() {
    for (auto ite = dataMap.begin(); ite != dataMap.end();) {
      int nodeKey = ite->first;
      Data *data = ite->second;
      // Deleting while iterating map as per
      // http://stackoverflow.com/a/8234813/1831275
      if (isJobFinished(nodeKey)) {
        dataMap.erase(ite++);
        //Free up memory
        delete data;
      } else {
        ++ite;
      }
    }
  }

  bool Scheduler::isJobFinished(int nodeKey) {
    lock();
    assertThatInvariantsHold();
    bool result = finished.find(nodeKey) != finished.end();
    assertThatInvariantsHold();
    unlock();
    return result;
  }

  Data *Scheduler::getDataByNodeKey(int nodeKey) {
    lock();
    assertThatInvariantsHold();
    assert(dataMap.find(nodeKey) != dataMap.end());
    Data *result = dataMap[nodeKey];
    assertThatInvariantsHold();
    unlock();
    return result;
  }

  Scheduler& Scheduler::getScheduler() {
    return Scheduler::INSTANCE;
  }

  // Privates
  void Scheduler::markJobFinished(int nodeKey) {
    lock();
    finished.insert(nodeKey);
    unlock();
  }

  void Scheduler::lock() {
    pthread_mutex_lock(&schedulerLock);
  }

  void Scheduler::unlock() {
    pthread_mutex_unlock(&schedulerLock);
  }

  void Scheduler::assertThatInvariantsHold() {
#ifdef DEBUG
    // Assert running jobs are not runnable and are not finished
    for (auto pair : runningJobs) {
      int nodeKey = pair.first;
      Node *node = pair.second;
      assert(Node::getNodeKeyByIds(node->nodeId, node->batchId) == nodeKey);
      assert(runnableJobs.find(node) == runnableJobs.end());
      assert(finished.find(nodeKey) == finished.end());

      // Assert graph does not contain running jobs
      assert(graph->nodes.find(nodeKey) == graph->nodes.end())
        }

    // Assert runnable jobs are not in graph
    for (Node *node : runnableJobs) {
      int nodeKey = Node::getNodeKeyByIds(node->nodeId, node->batchId);
      assert(graph->nodes.find(nodeKey) == graph->nodes.end());
    }

    // Assert keys in dataMap are all finished
    for (auto pair: dataMap) {
      int nodeKey = pair.first;
      assert(finished.find(nodeKey) != finished.end());
    }

#endif
  }

}
