#ifndef ELIXIR_WORKER_H
#define ELIXIR_WORKER_H

#include "graph.h"

#include <pthread.h>

namespace elixir {

  using namespace std;

  class Worker {

  int workerId;

  public:
    Worker(int id) {
      this->workerId = id;
    }

    void workerThread();

    static int getWorkerId() {
      pthread_mutex_lock(&tidMapLock);
      int result = tidMap[pthread_self()];
      pthread_mutex_unlock(&tidMapLock);
      return result;
    }

    // Map: tid -> worker Id
    static unordered_map<pthread_t, int> tidMap;

    static pthread_mutex_t tidMapLock;
  };
}

#endif /* ELIXIR_WORKER_H */
