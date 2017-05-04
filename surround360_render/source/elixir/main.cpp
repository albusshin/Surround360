#include "scheduler.h"

using namespace elixir;
using namespace std;

Graph *loadGraph() {
  //TODO implement
  // return a pointer on HEAP!!!!!!!!!!!!!

}

int main() {
  //TODO implement
  // build graph
  Graph *graph = loadGraph();
  Scheduler::getScheduler().init(graph);

  // spawn worker threads

  //
}
