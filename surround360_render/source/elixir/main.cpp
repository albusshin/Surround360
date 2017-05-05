#include "scheduler.h"
#include "graph.h"

using namespace elixir;
using namespace std;

Graph *loadGraph() {
  // TODO implement
  // return a pointer on HEAP!!!!!!!!!!!!!

  size_t frameNum = 10;
  size_t nodeNum = 44;

  // Create a graph object
  Graph *graph = new Graph(nodeNum, frameNum);

  // Add P nodes
  for (int i = 0; i < 14; ++i) {
    // Create parent list, child list
    vector<int> parent; // Empty
    vector<int> children;

    // Children: right f => left f => right r => left r
    children.push_back(i + 14);
    if (i == 0) {
      children.push_back(i + 27);
    } else {
      children.push_back(i + 13);
    }

    children.push_back(i + 28);
    if (i == 0) {
      children.push_back(i + 41);
    } else {
      children.push_back(i + 27);
    }

    // Create a node
    Node *node = new Node(i, 0, graph, parent, children);

    // TODO: CREATE KERNEL AND ADD TO NODE

    // Add to node list
    graph->nodes[node->getNodeKeyByIds(node->nodeId, node->batchId)] = node;
  }

  // Add F nodes
  for (int i = 14; i < 28; ++i) {
    // Create parent list, child list
    vector<int> parent;
    vector<int> children;

    // Parent sequence: left p => right p => previous f
    parent.push_back(i - 14);
    if (i == 27) {
      parent.push_back(i - 27);
    } else {
      parent.push_back(i - 13);
    }
    parent.push_back(i - 44);

    // Children sequence: r => later f
    children.push_back(i + 14);
    children.push_back(i + 44);

    // Create a node
    Node *node = new Node(i, 0, graph, parent, children);

    // TODO: CREATE KERNEL AND ADD TO NODE

    // Add to node list
    graph->nodes[node->getNodeKeyByIds(node->nodeId, node->batchId)] = node;
  }

  // Add R nodes
  for (int i = 28; i < 42; ++i) {
    // Create parent list, child list
    vector<int> parent;
    vector<int> children;

    // Parent sequence: left p => right p => f
    parent.push_back(i - 28);
    if (i == 28) {
      parent.push_back(i - 41);
    } else {
      parent.push_back(i - 27);
    }

    parent.push_back(i - 14);

    // Children sequence: left c => right c
    children.push_back(i + 14);
    children.push_back(i + 15);

    // Create a node
    Node *node = new Node(i, 0, graph, parent, children);

    // TODO: CREATE KERNEL AND ADD TO NODE

    // Add to node list
    graph->nodes[node->getNodeKeyByIds(node->nodeId, node->batchId)] = node;
  }

  // Add C nodes
  for (int i = 42; i < 44; ++i) {
    // Create parent list, child list
    vector<int> parent;
    vector<int> children; // Empty

    // Parent sequence: all r from 28 to 41
    for (int j = 18; j < 42; ++j) {
      parent.push_back(j);
    }

    // Create a node
    Node *node = new Node(i, 0, graph, parent, children);

    // TODO: CREATE KERNEL AND ADD TO NODE

    // Add to node list
    graph->nodes[node->getNodeKeyByIds(node->nodeId, node->batchId)] = node;
  }

  return graph;
}

int main() {
  //TODO implement
  // build graph
  Graph *graph = loadGraph();
  Scheduler::getScheduler().init(graph);

  // spawn worker threads

  //
}
