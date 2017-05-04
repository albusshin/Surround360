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
    pthread_mutex_t lock;

  public:
    void init(Graph *graph);

    Node *scheduleJob(int workerId);

    bool allFinished();

    void onJobFinishing(int nodeKey);

    bool isJobFinished(int nodeKey);

    void addData(Data *data);

    Data *readData(int nodeKey);

    static const Scheduler& getScheduler();

  private:

    Scheduler() {}

    // the priority queue
    unordered_set<Node *> runnableJobs;

    // Map: workerId -> running job
    unordered_map<int, Node *> runningJobs;

    Graph *graph;

    // Finished jobs. Key: NodeKey
    unordered_set<int> finished;

    unordered_map<int, Data *> dataMap;

    void markJobFinished(int nodeKey);

    void deleteData(int nodeKey);

    void setGraph(Graph *graph);

    static const Scheduler INSTANCE;
  };

}

#endif /* ELIXIR_SCHEDULER_H */
