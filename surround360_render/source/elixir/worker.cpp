#include <string>
#include <unistd.h>

#include "worker.h"
#include "scheduler.h"

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

      Data *outputData = node->kernel->execute(dataList);

      Scheduler::getScheduler().onJobFinishing(
        Node::getNodeKeyByIds(node->nodeId, node->batchId),
        outputData,
        workerId);
    }
  }
}
