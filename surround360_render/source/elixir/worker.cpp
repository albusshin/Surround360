#include "worker.h"
#include "scheduler.h"

#include <string>
#include <unistd.h>
#include <unordered_map>

namespace elixir {

  using namespace std;

  void Worker::workerThread() {

    while (true) {

      Node *node = Scheduler::getScheduler().scheduleJob(workerId);
      if (node == nullptr) {
        if (Scheduler::getScheduler().allFinished()) {
          break;
        } else {
          sleep(1);
        }
      }

      // get datalist and add data to datalist
      vector<Data *> dataList;
      for (int parentNodeKey : node->parents) {
        dataList.push_back(Scheduler::getScheduler().dataMap[parentNodeKey]);
      }

      unordered_map<string, void *> outputRawData = node->kernel->execute(dataList);

      Data *outputData = new Data(outputRawData,
                                  Node::getNodeKeyByIds(node->nodeId,
                                                        node->batchId),
                                  node->children);

      Scheduler::getScheduler().onJobFinishing(
        Node::getNodeKeyByIds(node->nodeId, node->batchId),
        outputData,
        workerId);
    }
  }
}
