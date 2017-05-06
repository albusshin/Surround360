#include "scheduler.h"
#include "graph.h"
#include "surround360_kernels.h"

#include <string>

using namespace elixir;
using namespace std;

size_t eqr_width = 8400;
size_t eqr_height = 8400;
string camera_rig_path = "/home/ubuntu/d/a/palace3/camera_rig.json";
string flow_algo = "pixflow_search_20";
float zero_parallax_dist = 10000;
float interpupilary_dist = 6.4;

Graph *loadGraph() {
  size_t frameNum = 1;
  size_t nodeNum = 58;

  // Create a graph object
  Graph *graph = new Graph(nodeNum, frameNum);

  // Add I nodes
  for (int i = 0; i < 14; ++i) {
    // Create parent list, child list
    vector<int> parent; // Empty
    vector<int> children;

    // Children: p
    children.push_back(i + 14);

    // Create a node
    Node *node = new Node(i, 0, graph, parent, children);

    // TODO: CREATE KERNEL

    // Add to node list
    graph->nodes[node->getNodeKeyByIds(node->nodeId, node->batchId)] = node;
  }

  // Add P nodes
  for (int i = 14; i < 28; ++i) {
    // Create parent list, child list
    vector<int> parent;
    vector<int> children;

    // Parent: p
    parent.push_back(i - 14);

    // Children: right f => left f => right r => left r
    children.push_back(i + 14);
    if (i == 14) {
      children.push_back(i + 27);
    } else {
      children.push_back(i + 13);
    }

    children.push_back(i + 28);
    if (i == 14) {
      children.push_back(i + 41);
    } else {
      children.push_back(i + 27);
    }

    // Create a node
    Node *node = new Node(i, 0, graph, parent, children);

    // TODO: cameraId == i for now.
    node->kernel = new KernelP(eqr_width, eqr_height, camera_rig_path, i);

    // Add to node list
    graph->nodes[node->getNodeKeyByIds(node->nodeId, node->batchId)] = node;
  }

  // Add F nodes
  for (int i = 28; i < 42; ++i) {
    // Create parent list, child list
    vector<int> parent;
    vector<int> children;

    // Parent sequence: left p => right p => previous f
    parent.push_back(i - 14);
    if (i == 41) {
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

    node->kernel = new KernelF(camera_rig_path, flow_algo);

    // Add to node list
    graph->nodes[node->getNodeKeyByIds(node->nodeId, node->batchId)] = node;
  }

  // Add R nodes
  for (int i = 42; i < 56; ++i) {
    // Create parent list, child list
    vector<int> parent;
    vector<int> children;

    // Parent sequence: left p => right p => f
    parent.push_back(i - 28);
    if (i == 42) {
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

    node->kernel = new KernelR(eqr_width,
                               eqr_height,
                               camera_rig_path,
                               flow_algo,
                               zero_parallax_dist,
                               interpupilary_dist);

    // Add to node list
    graph->nodes[node->getNodeKeyByIds(node->nodeId, node->batchId)] = node;
  }

  // Add C nodes
  for (int i = 56; i < 58; ++i) {
    // Create parent list, child list
    vector<int> parent;
    vector<int> children; // Empty

    // Parent sequence: all r from 28 to 41
    for (int j = 42; j < 56; ++j) {
      parent.push_back(j);
    }

    // Create a node
    Node *node = new Node(i, 0, graph, parent, children);

    node->kernel = new KernelC(eqr_width,
                               eqr_height,
                               camera_rig_path,
                               flow_algo,
                               zero_parallax_dist,
                               interpupilary_dist,
                               i == 56); //If i == 42 then it's left, otherwise it's right.

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
