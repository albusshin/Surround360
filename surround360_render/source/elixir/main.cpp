#include "scheduler.h"
#include "graph.h"
#include "surround360_kernels.h"
#include "worker.h"

#include <pthread.h>
#include <string>

using namespace elixir;
using namespace std;

size_t eqr_width = 8400;
size_t eqr_height = 8400;
string camera_rig_path = "/home/ubuntu/d/a/palace3/camera_rig.json";
string flow_algo = "pixflow_search_20";
float zero_parallax_dist = 10000;
float interpupilary_dist = 6.4;

string get_video_filename(int camId) {
  assert(camId >= 0 && camId <= 16);
  std::stringstream ss;
  ss << "/home/ubuntu/d/a/palace3/rgb/cam" << camId << "/vid.mp4";
  return ss.str();
}

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
    elixir::Node *node = new elixir::Node(i, 0, graph, parent, children);

    node->kernel = new KernelI(get_video_filename(i), 0);

    // Add to node list
    graph->nodes[node->getNodeKeyByIds(node->nodeId, node->batchId)] = node;
  }

  int offset = 1;
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
    elixir::Node *node = new elixir::Node(i, 0, graph, parent, children);

    int camIdx = ((i - 14) + offset) % 14;

    node->kernel = new KernelP(eqr_width, eqr_height, camera_rig_path, camIdx);

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
    parent.push_back(i - 58);

    vector<int> dummyNextLayer;
    dummyNextLayer.push_back(i);
    unordered_map<string, void *> dummyRawData;
    dummyRawData["prev_overlap_image_l_"] = (void *) new cv::Mat();
    dummyRawData["prev_overlap_image_r_"] = (void *) new cv::Mat();
    dummyRawData["prev_frame_flow_l_to_r_"] = (void *) new cv::Mat();
    dummyRawData["prev_frame_flow_r_to_l_"] = (void *) new cv::Mat();
    dummyRawData["left_flow"] = (void *) new cv::Mat();
    dummyRawData["right_flow"] = (void *) new cv::Mat();
    Data *dummyData = new Data(dummyRawData,
                               i - 58,
                               dummyNextLayer);
    Scheduler::getScheduler().addDummyData(i - 58, dummyData);

    // Children sequence: r => later f
    children.push_back(i + 14);
    children.push_back(i + 58);

    // Create a node
    elixir::Node *node = new elixir::Node(i, 0, graph, parent, children);

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
    elixir::Node *node = new elixir::Node(i, 0, graph, parent, children);

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
    elixir::Node *node = new elixir::Node(i, 0, graph, parent, children);

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

#define NUM_CORES 32

void *worker_thread(void *arg) {
  int tid = *(int *) arg;
  std::cout << "[T " << tid << "]\t"
            << " Up and running."
            << std::endl;

  elixir::Worker *worker = new Worker(tid);
  worker->workerThread();

  std::cout << "[T " << tid << "]\t"
            << "Finished. "
            << std::endl;
}

Graph *theGraph;
Scheduler *theScheduler;

int main() {
  // build graph
  Graph *graph = loadGraph();
  theGraph = graph;
  Scheduler::getScheduler().init(graph);
  theScheduler = &(Scheduler::getScheduler());

  pthread_t threads[NUM_CORES];

  int ids[NUM_CORES];
  // spawn worker threads
  for (int i = 0; i < 32; ++i) {
    ids[i] = i;
    pthread_create(&threads[i], NULL, worker_thread, &ids[i]);
  }

  // join worker threads
  for (int i = 0; i < 32; ++i) {
    pthread_join(threads[i], NULL);
  }

  std::cout << "[Main]\t"
            << "Joined all threads. Exiting."
            << std::endl;

  return 0;
}
