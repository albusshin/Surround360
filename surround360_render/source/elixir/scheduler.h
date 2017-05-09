#ifndef ELIXIR_SCHEDULER_H
#define ELIXIR_SCHEDULER_H

#include <pthread.h>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <string>
#include <list>

#include "graph.h"

namespace elixir {

  using namespace std;

  class Scheduler {
    pthread_mutex_t schedulerLock;
    pthread_cond_t schedulerCondVar;

  public:
    void printRunnableJobs();

    void init(Graph *graph);

    Node *scheduleJob(int workerId);

    bool allFinished();

    void onJobFinishing(int nodeKey, Data *outputData, int workerId);

    void addDummyData(int dummyNodeKey, Data *dummyData);

    bool isJobFinished(int nodeKey);

    Data *getDataByNodeKey(int nodeKey);

    bool isJobBatchTooDeep(int batchId);

    static Scheduler& getScheduler();

    Graph *graph;

    // Map: nodeKey -> data output of that job
    unordered_map<int, Data *> dataMap;

  private:

    Scheduler() {
        pthread_mutex_init(&schedulerLock, NULL);
        pthread_cond_init(&schedulerCondVar, NULL);
    }
    ~Scheduler() {
        pthread_mutex_destroy(&schedulerLock);
        pthread_cond_destroy(&schedulerCondVar);
    }

    // the priority queue
    list<Node *> runnableJobs;

    // Map: workerId -> running job
    unordered_map<int, Node *> runningJobs;

    // Finished jobs. Key: NodeKey
    unordered_set<int> finished;

    enum SchedulerPolicy { Fifo, Optimized };

    static SchedulerPolicy policy;

    const int LAYERS_THRESHOLD = 3;

    void lock();

    void unlock();

    void dataMapCleanup();

    void markJobFinished(int nodeKey);

    void getJobsWithMinBatchID(std::map<int, Node *> &jobs);

    void getJobsWithMinDepth(std::map<int, Node *> &jobs);

    void getJobsWithLocality(std::map<int, Node *> &jobs, int workerId);

    void getJobsWithMaxChildren(std::map<int, Node *> &jobs);

    int getFirstComeJob(std::map<int, Node *> &jobs);

    Node *fifoPickAJob(int workerId);

    Node *optimizedPickAJob(int workerId);

    Node *pickAJob(int workerId);
    
    void assertThatInvariantsHold();

    static Scheduler INSTANCE;
  };

}

#endif /* ELIXIR_SCHEDULER_H */
