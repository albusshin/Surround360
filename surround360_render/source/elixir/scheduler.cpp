#include <iostream>
#include <pthread.h>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include <assert.h>
#include <stdexcept>
#include <limits.h>

#include "scheduler.h"
#include "graph.h"
#include "nullbuf.h"

#define DEBUG

//#define logger cout
#define logger null_stream


namespace elixir {

  using namespace std;

  enum Scheduler::SchedulerPolicy Scheduler::policy;
  Scheduler Scheduler::INSTANCE;

  void Scheduler::init(Graph *graph) {
    this->graph = graph;
    this->policy = SchedulerPolicy::Fifo;
  }

  Node *Scheduler::fifoPickAJob(int workerId) {
    Node *node = NULL;
    if (this->runnableJobs.size() != 0) {
      node = this->runnableJobs.front();
      this->runnableJobs.pop_front();
    }
    return node;
  }

  Node *Scheduler::optimizedPickAJob(int workerId) {
    // TODO: implementation
    return NULL;
  }

  Node *Scheduler::pickAJob(int workerId) {
    Node* node = NULL;

    switch(this->policy) {
    case SchedulerPolicy::Fifo:
      node = fifoPickAJob(workerId);
      break;
    case SchedulerPolicy::Optimized:
      node = optimizedPickAJob(workerId);
      break;
    default:
      throw invalid_argument("Invalid Policy!!\n");
    }

    return node;
  }

  Node *Scheduler::scheduleJob(int workerId) {
    lock();
    assertThatInvariantsHold();
    // parse graph to find every runnable job
    vector<Node *> runnableJobs = this->graph->getRunnableJobs();
    logger << "[Scheduler]\t"
           << "workerId: " << workerId
           << " runnableJobs.size() == "
           << runnableJobs.size()
           << endl;

    for (vector<Node *>::iterator it = runnableJobs.begin();
         it != runnableJobs.end(); ++it) {
      this->runnableJobs.push_back(*it);
    }

    // select job from job queue
    // Delete the node from the runnableQueue
    Node *node = pickAJob(workerId);

    if (node != nullptr) {
      // Add the node to the runningMap
      this->runningJobs[workerId] = node;
    }

    assertThatInvariantsHold();
    unlock();
    return node;
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
    assert(graph->nodes.find(nodeKey) == graph->nodes.end());
    Node *finishingNode = runningJobs[workerId];
    assert(finishingNode != nullptr);

    // remove from runningJobs
    runningJobs.erase(workerId);

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
    assertThatInvariantsHold();
    if (nodeKey < 0) {
      // treat all nodes with nodeKey less than 0 as finished
      return true;
    } else {
      bool result = finished.find(nodeKey) != finished.end();
      assertThatInvariantsHold();
      return result;
    }
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

  bool Scheduler::isJobBatchTooDeep(int batchId) {
    const int LAYERS_THRESHOLD = 1;

    if (runnableJobs.empty()) {
      return false;
    } else {
      int minBatchIdInRunnableQueue = INT_MAX;
      for (Node *node: runnableJobs) {
        minBatchIdInRunnableQueue = min(node->batchId, minBatchIdInRunnableQueue);
      }
      return minBatchIdInRunnableQueue + LAYERS_THRESHOLD < batchId;
    }
  }

  Scheduler& Scheduler::getScheduler() {
    return Scheduler::INSTANCE;
  }

  // Privates
  void Scheduler::markJobFinished(int nodeKey) {
    finished.insert(nodeKey);
  }

  void Scheduler::lock() {
    pthread_t tid = pthread_self();
    logger << "[Scheduler]\t"
           << tid
           << ": lock()"
           << endl;
    pthread_mutex_lock(&schedulerLock);
    logger << "[Scheduler]\t"
           << tid
           << ": acquired lock."
           << endl;
  }

  void Scheduler::unlock() {
    pthread_t tid = pthread_self();
    logger << "[Scheduler]\t"
           << tid
           << ": unlock()"
           << endl;
    pthread_mutex_unlock(&schedulerLock);
  }

  void Scheduler::assertThatInvariantsHold() {
#ifdef DEBUG
    // Assert running jobs are not runnable and are not finished
    for (auto pair : runningJobs) {
      Node *node = pair.second;
      assert(node != nullptr);
      int nodeKey = Node::getNodeKeyByIds(node->nodeId, node->batchId);
      for (Node *runnableJob : runnableJobs) {
        assert(node != runnableJob);
        assert(
          Node::getNodeKeyByIds(node->nodeId, node->batchId)
          != Node::getNodeKeyByIds(runnableJob->nodeId, runnableJob->batchId));
      }
      assert(finished.find(nodeKey) == finished.end());

      // Assert graph does not contain running jobs
      assert(graph->nodes.find(nodeKey) == graph->nodes.end());
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
