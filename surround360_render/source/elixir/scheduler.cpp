#include <pthread.h>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>

#include "scheduler.h"
#include "graph.h"

namespace elixir {

  using namespace std;

  void Scheduler::init(Graph *graph) {
    //TODO implement
    this->setGraph(graph);

    // parse graph to find every runnable job
    vector<Node *> runnableJobs = this->graph->getRunnableJobs();
    for (Node *node : runnableJobs) {
      this->runnableJobs.insert(node);
    }
  }

  Node *Scheduler::scheduleJob(int workerId) {
    //TODO implement
    // select job from job queue

  }

  bool Scheduler::allFinished() {
    //TODO implement
  }

  void Scheduler::onJobFinishing(int nodeKey) {
    //TODO implement
    // update state (graph, data, ...)
    // set job finished
    // cleanup ()

  }

  bool Scheduler::isJobFinished(int nodeKey) {
    //TODO implement
    return finished.find(nodeKey) != finished.end();
  }

  void Scheduler::addData(Data *data) {
    //TODO implement

  }

  Data *Scheduler::readData(int nodeKey) {
    //TODO implement

  }

  const Scheduler& Scheduler::getScheduler() {
    //TODO implement
    return Scheduler::INSTANCE;
  }

  // Privates
  void Scheduler::markJobFinished(int nodeKey) {
    //TODO implement
    finished.insert(nodeKey);
  }

  void Scheduler::deleteData(int nodeKey) {
    //TODO implement

  }

  void Scheduler::setGraph(Graph *graph) {
    //TODO implement
    this->graph = graph;
  }

};

}

elixir::Graph *loadGraph() {
  // return a pointer on HEAP!!!!!!!!!!!!!

}

int main() {
  elixir::Scheduler scheduler;

  // build graph
  elixir::Graph *graph = loadGraph();
  scheduler.init(graph);

  // spawn worker threads

  //
}
