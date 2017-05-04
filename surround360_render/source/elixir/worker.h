#ifndef ELIXIR_WORKER_H
#define ELIXIR_WORKER_H

#include "graph.h"

namespace elixir {

  using namespace std;

  class Worker {

    int workerId;

  public:
    Worker(int id) {
      this->workerId = id;
    }

    void workerThread();
  };
}

#endif /* ELIXIR_WORKER_H */
