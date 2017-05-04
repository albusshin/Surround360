#include <string>

#include "worker.h"
#include "scheduler.h"

namespace elixir {

  using namespace std;

  void Worker::workerThread() {
    //TODO implement

    while (true) {

      Node *node = Scheduler::getScheduler().scheduleJob(workerId);
      if (node == nullptr && Scheduler::getScheduler().allFinished()) {
        break;
      }

      // get datalist

      // node.kernel.execute(datalist);

      // scheduler.finishJob();

    }
  }

}
