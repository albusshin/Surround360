#include "worker.h"
#include "scheduler.h"

#include <assert.h>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include "nullbuf.h"

#define DEBUG

#define logger cout
//#define logger null_stream

namespace elixir {

  using namespace std;

  unordered_map<pthread_t, int> Worker::tidMap;

  pthread_mutex_t Worker::tidMapLock;

  void Worker::workerThread() {

    while (true) {

      Node *node = Scheduler::getScheduler().scheduleJob(workerId);
      if (node == nullptr) {
        break;
      }

      // get datalist and add data to datalist
      vector<Data *> dataList;
      for (int parentNodeKey : node->parents) {
        dataList.push_back(Scheduler::getScheduler().getDataByNodeKey(parentNodeKey));
      }

      unordered_map<string, void *> outputRawData = node->kernel->execute(dataList);

      Data *outputData = new Data(outputRawData,
                                  Node::getNodeKeyByIds(node->nodeId,
                                                        node->batchId),
                                  node->children);

      logger << "[Worker"
             << workerId
             << "]\t"
             << "Work done."
             << endl;

      Scheduler::getScheduler().onJobFinishing(
        Node::getNodeKeyByIds(node->nodeId, node->batchId),
        outputData,
        workerId);

      logger << "[Worker"
             << workerId
             << "]\t"
             << "After onJobFinishing"
             << endl;
    }
  }
}
