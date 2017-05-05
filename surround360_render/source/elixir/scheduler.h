#ifndef ELIXIR_SCHEDULER_H
#define ELIXIR_SCHEDULER_H

#include <pthread.h>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>

#include "graph.h"

namespace elixir {

  using namespace std;

  class Scheduler {
    pthread_mutex_t schedulerLock;

  public:
    void init(Graph *graph);

    Node *scheduleJob(int workerId);

    bool allFinished();

    void onJobFinishing(int nodeKey, Data *outputData, int workerId);

    bool isJobFinished(int nodeKey);

    Data *getDataByNodeKey(int nodeKey);

    static Scheduler& getScheduler();

    Graph *graph;

    // Map: nodeKey -> data output of that job
    unordered_map<int, Data *> dataMap;

  private:

    Scheduler() {}

    // the priority queue
    unordered_set<Node *> runnableJobs;

    // Map: workerId -> running job
    unordered_map<int, Node *> runningJobs;

    // Finished jobs. Key: NodeKey
    unordered_set<int> finished;

    void lock();

    void unlock();

    void dataMapCleanup();

    void markJobFinished(int nodeKey);
    
    void assertThatInvariantsHold();

    static Scheduler INSTANCE;
  };

}

#endif /* ELIXIR_SCHEDULER_H */
